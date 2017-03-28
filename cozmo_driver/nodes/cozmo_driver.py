#!/usr/bin/python3.5
# -*- encoding: utf-8 -*-
"""
This file implements an ANKI Cozmo ROS driver.

It wraps up several functionality of the Cozmo SDK including
camera and motors. As some main ROS parts are not python3.5
compatible, the famous "transformations.py" is shipped next
to this node. Also the TransformBroadcaster is taken from
ROS tf ones.

Copyright {2016} {Takashi Ogura}
Copyright {2017} {Peter Rudolph}

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

"""
# system
import sys
import numpy as np
from copy import deepcopy

# cozmo SDK
import cozmo
from cozmo.util import radians

# ROS
import rospy
from transformations import quaternion_from_euler, quaternion_matrix, quaternion_from_matrix
from camera_info_manager import CameraInfoManager

# ROS msgs
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    Twist,
    TransformStamped
)
from std_msgs.msg import (
    String,
    Float64,
    ColorRGBA,
)
from sensor_msgs.msg import (
    Image,
    CameraInfo,
    BatteryState,
    Imu,
    JointState,
)


# reused as original is not Python3 compatible
class TransformBroadcaster(object):
    """
    :class:`TransformBroadcaster` is a convenient way to send transformation updates on the ``"/tf"`` message topic.
    """

    def __init__(self, queue_size=100):
        self.pub_tf = rospy.Publisher("/tf", TFMessage, queue_size=queue_size)

    def send_transform(self, translation, rotation, time, child, parent):
        """
        :param translation: the translation of the transformation as a tuple (x, y, z)
        :param rotation: the rotation of the transformation as a tuple (x, y, z, w)
        :param time: the time of the transformation, as a rospy.Time()
        :param child: child frame in tf, string
        :param parent: parent frame in tf, string

        Broadcast the transformation from tf frame child to parent on ROS topic ``"/tf"``.
        """

        t = TransformStamped()
        t.header.frame_id = parent
        t.header.stamp = time
        t.child_frame_id = child
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]

        t.transform.rotation.x = rotation[0]
        t.transform.rotation.y = rotation[1]
        t.transform.rotation.z = rotation[2]
        t.transform.rotation.w = rotation[3]

        self.send_transform_message(t)

    def send_transform_message(self, transform):
        """
        :param transform: geometry_msgs.msg.TransformStamped
        Broadcast the transformation from tf frame child to parent on ROS topic ``"/tf"``.
        """
        tfm = TFMessage([transform])
        self.pub_tf.publish(tfm)


class CozmoRos(object):
    """
    The Cozmo ROS driver object.

    """
    
    def __init__(self, coz):
        """

        :type   coz:    cozmo.Robot
        :param  coz:    The cozmo SDK robot handle (object).
        
        """

        # vars
        self._cozmo = coz
        self._lin_vel = .0
        self._ang_vel = .0
        self._cmd_lin_vel = .0
        self._cmd_ang_vel = .0
        self._last_pose = self._cozmo.pose
        self._wheel_vel = (0, 0)
        self._optical_frame_orientation = quaternion_from_euler(-np.pi/2., .0, -np.pi/2.)
        self._camera_info_manager = CameraInfoManager('cozmo_camera', namespace='/cozmo_camera')
        self._last_seen_cube = []
        self.cubes_visible = 0

        # tf
        self._tfb = TransformBroadcaster()

        # params
        self._odom_frame = rospy.get_param('~odom_frame', 'odom')
        self._footprint_frame = rospy.get_param('~footprint_frame', 'base_footprint')
        self._base_frame = rospy.get_param('~base_frame', 'base_link')
        self._head_frame = rospy.get_param('~head_frame', 'head_link')
        self._camera_frame = rospy.get_param('~camera_frame', 'camera_link')
        self._camera_optical_frame = rospy.get_param('~camera_optical_frame', 'cozmo_camera')
        camera_info_url = rospy.get_param('~camera_info_url', '')

        # pubs
        self._joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
        self._odom_pub = rospy.Publisher('odom', Odometry, queue_size=1)
        self._imu_pub = rospy.Publisher('imu', Imu, queue_size=1)
        self._battery_pub = rospy.Publisher('battery', BatteryState, queue_size=1)
        # Note: camera is published under global topic (preceding "/")
        self._image_pub = rospy.Publisher('/cozmo_camera/image', Image, queue_size=10)
        self._camera_info_pub = rospy.Publisher('/cozmo_camera/camera_info', CameraInfo, queue_size=10)

        # subs
        self._backpack_led_sub = rospy.Subscriber(
            'backpack_led', ColorRGBA, self._set_backpack_led, queue_size=1)
        self._twist_sub = rospy.Subscriber('cmd_vel', Twist, self._twist_callback, queue_size=1)
        self._say_sub = rospy.Subscriber('say', String, self._say_callback, queue_size=1)
        self._head_sub = rospy.Subscriber('head_angle', Float64, self._move_head, queue_size=1)
        self._lift_sub = rospy.Subscriber('lift_height', Float64, self._move_lift, queue_size=1)

        # camera info manager
        self._camera_info_manager.setURL(camera_info_url)
        self._camera_info_manager.loadCameraInfo()

    def _move_head(self, cmd):
        """
        Move head to given angle.
        
        :type   cmd:    Float64
        :param  cmd:    The message containing angle in degrees. [-25 - 44.5]
        
        """
        action = self._cozmo.set_head_angle(radians(cmd.data * np.pi / 180.), duration=0.0,
                                            in_parallel=True)
        action.wait_for_completed()

    def _move_lift(self, cmd):
        """
        Move lift to given height.

        :type   cmd:    Float64
        :param  cmd:    A value between [0 - 1], the SDK auto
                        scales it to the according height.

        """
        action = self._cozmo.set_lift_height(height=cmd.data,
                                             duration=0.0, in_parallel=True)
        action.wait_for_completed()

    def _set_backpack_led(self, msg):
        """
        Set the color of the backpack LEDs.

        :type   msg:    ColorRGBA
        :param  msg:    The color to be set.

        """
        # setup color as integer values
        color = [int(x * 255) for x in [msg.r, msg.g, msg.b, msg.a]]
        # create lights object with duration
        light = cozmo.lights.Light(cozmo.lights.Color(rgba=color), on_period_ms=1000)
        # set lights
        self._cozmo.set_all_backpack_lights(light)

    def _twist_callback(self, cmd):
        """
        Set commanded velocities from Twist message.

        The commands are actually send/set during run loop, so delay
        is in worst case up to 1 / update_rate seconds.

        :type   cmd:    Twist
        :param  cmd:    The commanded velocities.

        """
        # compute differential wheel speed
        axle_length = 0.07  # 7cm
        self._cmd_lin_vel = cmd.linear.x
        self._cmd_ang_vel = cmd.angular.z
        rv = self._cmd_lin_vel + (self._cmd_ang_vel * axle_length * 0.5)
        lv = self._cmd_lin_vel - (self._cmd_ang_vel * axle_length * 0.5)
        self._wheel_vel = (lv*1000., rv*1000.)  # convert to mm / s

    def _say_callback(self, msg):
        """
        The callback for incoming text messages to be said.

        :type   msg:    String
        :param  msg:    The text message to say.

        """
        self._cozmo.say_text(msg.data, in_parallel=True).wait_for_completed()

    def _publish_objects(self):
        """
        Publish detected object as transforms between odom_frame and object_frame.

        """
        self.cubes_visible = 0
        for obj in self._cozmo.world.visible_objects:
            self.cubes_visible += 1
            now = rospy.Time.now()
            x = obj.pose.position.x * 0.001
            y = obj.pose.position.y * 0.001
            z = obj.pose.position.z * 0.001
            q = (obj.pose.rotation.q1, obj.pose.rotation.q2, obj.pose.rotation.q3, obj.pose.rotation.q0)
            # self._tfb.send_transform(
                # (x, y, z), q, now, 'cube_' + str(obj.object_id), self._odom_frame
                # (x, y, z), q, now, 'world', self._odom_frame
            # )
            self._last_seen_cube = [(x,y,z),q,now]

        # if len(self._last_seen_cube) != 0:
            # print(self._last_seen_cube)
            # now = rospy.Time.now()
            # self._tfb.send_transform(
            #     # (x, y, z), q, now, 'cube_' + str(obj.object_id), self._odom_frame
            #     self._last_seen_cube[0], self._last_seen_cube[1], now, self._odom_frame, 'world'
                    
            # )      



        # if len(self._last_seen_cube) == 0:
        #     now = rospy.Time.now()
        #     self._tfb.send_transform(
        #             # (x, y, z), q, now, 'cube_' + str(obj.object_id), self._odom_frame

        #             (self.last_cube_x, self.last_cube_y, self.last_cube_z), self.last_cube_q, now, self._odom_frame, 'world'
        #         )
        # else:
        #     for obj in self._cozmo.world.visible_objects:
        #         now = rospy.Time.now()
        #         x = obj.pose.position.x * 0.001
        #         y = obj.pose.position.y * 0.001
        #         z = obj.pose.position.z * 0.001
        #         q = (obj.pose.rotation.q1, obj.pose.rotation.q2, obj.pose.rotation.q3, obj.pose.rotation.q0)
        #         self._tfb.send_transform(
        #             (x, y, z), q, now, 'cube_' + str(obj.object_id), self._odom_frame
        #             # (x, y, z), q, now, 'world', self._odom_frame
        #         )

        #         self.last_cube_x = x
        #         self.last_cube_y = y
        #         self.last_cube_z = z
        #         self.last_cube_q = q




    def _publish_image(self):
        """
        Publish latest camera image as Image with CameraInfo.

        """
        # only publish if we have a subscriber
        if self._image_pub.get_num_connections() == 0:
            return

        # get latest image from cozmo's camera
        camera_image = self._cozmo.world.latest_image
        if camera_image is not None:
            # convert image to gray scale as it is gray although
            img = camera_image.raw_image.convert('L')
            ros_img = Image()
            ros_img.encoding = 'mono8'
            ros_img.width = img.size[0]
            ros_img.height = img.size[1]
            ros_img.step = ros_img.width
            ros_img.data = img.tobytes()
            ros_img.header.frame_id = 'cozmo_camera'
            cozmo_time = camera_image.image_recv_time
            ros_img.header.stamp = rospy.Time.from_sec(cozmo_time)
            # publish images and camera info
            self._image_pub.publish(ros_img)
            camera_info = self._camera_info_manager.getCameraInfo()
            camera_info.header = ros_img.header
            self._camera_info_pub.publish(camera_info)

    def _publish_joint_state(self):
        """
        Publish joint states as JointStates.

        """
        # only publish if we have a subscriber
        if self._joint_state_pub.get_num_connections() == 0:
            return

        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.header.frame_id = 'cozmo'
        js.name = ['head', 'lift']
        js.position = [self._cozmo.head_angle.radians,
                       self._cozmo.lift_height.distance_mm * 0.001]
        js.velocity = [0.0, 0.0]
        js.effort = [0.0, 0.0]
        self._joint_state_pub.publish(js)

    def _publish_imu(self):
        """
        Publish inertia data as Imu message.

        """
        # only publish if we have a subscriber
        if self._imu_pub.get_num_connections() == 0:
            return

        imu = Imu()
        imu.header.stamp = rospy.Time.now()
        imu.header.frame_id = self._base_frame
        imu.orientation.w = self._cozmo.pose.rotation.q0
        imu.orientation.x = self._cozmo.pose.rotation.q1
        imu.orientation.y = self._cozmo.pose.rotation.q2
        imu.orientation.z = self._cozmo.pose.rotation.q3
        imu.angular_velocity.x = self._cozmo.gyro.x
        imu.angular_velocity.y = self._cozmo.gyro.y
        imu.angular_velocity.z = self._cozmo.gyro.z
        imu.linear_acceleration.x = self._cozmo.accelerometer.x * 0.001
        imu.linear_acceleration.y = self._cozmo.accelerometer.y * 0.001
        imu.linear_acceleration.z = self._cozmo.accelerometer.z * 0.001
        self._imu_pub.publish(imu)

    def _publish_battery(self):
        """
        Publish battery as BatteryState message.

        """
        # only publish if we have a subscriber
        if self._battery_pub.get_num_connections() == 0:
            return

        battery = BatteryState()
        battery.header.stamp = rospy.Time.now()
        battery.voltage = self._cozmo.battery_voltage
        battery.present = True
        if self._cozmo.is_on_charger:  # is_charging always return False
            battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        else:
            battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        self._battery_pub.publish(battery)

    def _publish_odometry(self):
        """
        Publish current pose as Odometry message.

        """
        # only publish if we have a subscriber
        if self._odom_pub.get_num_connections() == 0:
            return

        now = rospy.Time.now()
        odom = Odometry()
        odom.header.frame_id = self._odom_frame
        odom.header.stamp = now
        odom.child_frame_id = self._footprint_frame
        odom.pose.pose.position.x = self._cozmo.pose.position.x * 0.001
        odom.pose.pose.position.y = self._cozmo.pose.position.y * 0.001
        odom.pose.pose.position.z = self._cozmo.pose.position.z * 0.001
        q = quaternion_from_euler(.0, .0, self._cozmo.pose_angle.radians)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.pose.covariance = np.diag([1e-2, 1e-2, 1e-2, 1e3, 1e3, 1e-1]).ravel()
        odom.twist.twist.linear.x = self._lin_vel
        odom.twist.twist.angular.z = self._ang_vel
        odom.twist.covariance = np.diag([1e-2, 1e3, 1e3, 1e3, 1e3, 1e-2]).ravel()
        self._odom_pub.publish(odom)

    
    def getWorldtoOdomTransform(self): #takes in robotToWorld and OdomToRobot and returns WorldToOdom

        x_odomToRobot = self._cozmo.pose.position.x * 0.001
        y_odomToRobot = self._cozmo.pose.position.y * 0.001
        z_odomToRobot = self._cozmo.pose.position.z * 0.001
        q_odomToRobot = quaternion_from_euler(.0, .0, self._cozmo.pose_angle.radians)

        x_robotToWorld = self._last_seen_cube[0][0]
        y_robotToWorld = self._last_seen_cube[0][1]
        z_robotToWorld = self._last_seen_cube[0][2]
        q_robotToWorld = self._last_seen_cube[1]

        R_odomToRobot = quaternion_matrix(q_odomToRobot)
        R_robotToWorld = quaternion_matrix(q_robotToWorld)

        # bottomRow = np.array([0, 0, 0, 1])
        
        # print(R_odomToRobot)
        # print(np.array([x_odomToRobot, y_odomToRobot, z_odomToRobot]).T)
        T_odomToRobot = np.zeros((4,4))
        T_odomToRobot[0] = np.array([R_odomToRobot[0][0], R_odomToRobot[0][1], R_odomToRobot[0][2], x_odomToRobot])
        T_odomToRobot[1] = np.array([R_odomToRobot[1][0], R_odomToRobot[1][1], R_odomToRobot[1][2], y_odomToRobot])
        T_odomToRobot[2] = np.array([R_odomToRobot[2][0], R_odomToRobot[2][1], R_odomToRobot[2][2], z_odomToRobot])
        T_odomToRobot[3][3] = 1
        print("odomToRobot", T_odomToRobot)
        # T_odomToRobot = np.concatenate((T_odomToRobot, bottomRow), axis = 0)
        # T_robotToWorld = np.array(R_robotToWorld)
        # T_robotToWorld[0:3][3] = np.array([x_robotToWorld, y_robotToWorld, z_robotToWorld])
        # T_robotToWorld = np.concatenate((T_robotToWorld, bottomRow), axis = 0)

        T_robotToWorld = np.zeros((4,4))
        T_robotToWorld[0] = np.array([R_robotToWorld[0][0], R_robotToWorld[0][1], R_robotToWorld[0][2], x_robotToWorld])
        T_robotToWorld[1] = np.array([R_robotToWorld[1][0], R_robotToWorld[1][1], R_robotToWorld[1][2], y_robotToWorld])
        T_robotToWorld[2] = np.array([R_robotToWorld[2][0], R_robotToWorld[2][1], R_robotToWorld[2][2], z_robotToWorld])
        T_robotToWorld[3][3] = 1
        print("robotToWorld", T_robotToWorld)
        # print(T_robotToWorld)

        T_worldToOdom = np.dot(np.linalg.inv(T_odomToRobot), np.linalg.inv(T_robotToWorld))
        print("worldToOdom", T_worldToOdom)
        # R_worldToOdom = T_worldToOdom[0:][0:]

        q = quaternion_from_matrix(T_worldToOdom)
        x = T_worldToOdom[0][3]
        y = T_worldToOdom[1][3]
        

        return [x, y, q]




    def _publish_tf(self, update_rate):
        """
        Broadcast current transformations and update
        measured velocities for odometry twist.

        Published transforms:

        odom_frame -> footprint_frame
        footprint_frame -> base_frame
        base_frame -> head_frame
        head_frame -> camera_frame
        camera_frame -> camera_optical_frame

        """
        now = rospy.Time.now()
        x = self._cozmo.pose.position.x * 0.001
        y = self._cozmo.pose.position.y * 0.001
        z = self._cozmo.pose.position.z * 0.001

        #publish world -> odom frame
        if len(self._last_seen_cube) != 0:
            
            if self.cubes_visible > 0:
                state = self.getWorldtoOdomTransform() #takes in robotToWorld and OdomToRobot and returns WorldToOdom
                q = state[2]
                now = rospy.Time.now()
                self._tfb.send_transform(
                    (state[0], state[1], 0.0), q, now, self._odom_frame, 'world')

                self.last_state = state
            
            else:
                state = self.last_state
                q = state[2]
                now = rospy.Time.now()
                self._tfb.send_transform(
                    (state[0], state[1], 0.0), q, now, self._odom_frame, 'world')                

            # print(state[0], state[1])


        # compute current linear and angular velocity from pose change
        # Note: Sign for linear velocity is taken from commanded velocities!
        # Note: The angular velocity can also be taken from gyroscopes!
        delta_pose = self._last_pose - self._cozmo.pose
        dist = np.sqrt(delta_pose.position.x**2
                       + delta_pose.position.y**2
                       + delta_pose.position.z**2) / 1000.0
        self._lin_vel = dist * update_rate * np.sign(self._cmd_lin_vel)
        self._ang_vel = -delta_pose.rotation.angle_z.radians * update_rate

        # publish odom_frame -> footprint_frame
        q = quaternion_from_euler(.0, .0, self._cozmo.pose_angle.radians)
        self._tfb.send_transform(
            (x, y, 0.0), q, now, self._footprint_frame, self._odom_frame)

        # publish footprint_frame -> base_frame
        q = quaternion_from_euler(.0, -self._cozmo.pose_pitch.radians, .0)
        self._tfb.send_transform(
            (0.0, 0.0, 0.02), q, now, self._base_frame, self._footprint_frame)

        # publish base_frame -> head_frame
        q = quaternion_from_euler(.0, -self._cozmo.head_angle.radians, .0)
        self._tfb.send_transform(
            (0.02, 0.0, 0.05), q, now, self._head_frame, self._base_frame)

        # publish head_frame -> camera_frame
        self._tfb.send_transform(
            (0.025, 0.0, -0.015), (0.0, 0.0, 0.0, 1.0), now, self._camera_frame, self._head_frame)

        # publish camera_frame -> camera_optical_frame
        q = self._optical_frame_orientation
        self._tfb.send_transform(
            (0.0, 0.0, 0.0), q, now, self._camera_optical_frame, self._camera_frame)

    



        # store last pose
        self._last_pose = deepcopy(self._cozmo.pose)

    def run(self, update_rate=60):
        """
        Publish data continuously with given rate.

        :type   update_rate:    int
        :param  update_rate:    The update rate.

        """
        r = rospy.Rate(update_rate)
        while not rospy.is_shutdown():
            self._publish_tf(update_rate)
            self._publish_image()
            self._publish_objects()
            self._publish_joint_state()
            self._publish_imu()
            self._publish_battery()
            self._publish_odometry()
            # send message repeatedly to avoid idle mode.
            # This might cause low battery soon
            # TODO improve this!
            self._cozmo.drive_wheels(*self._wheel_vel)
            # sleep
            r.sleep()
        # stop events on cozmo
        self._cozmo.stop_all_motors()


def cozmo_app(coz_conn):
    """
    The main function of the cozmo ROS driver.

    This function is called by cozmo SDK!
    Use "cozmo.connect(cozmo_app)" to run.

    :type   coz_conn:   cozmo.Connection
    :param  coz_conn:   The connection handle to cozmo robot.

    """
    coz = coz_conn.wait_for_robot()
    coz.camera.image_stream_enabled = True
    coz_ros = CozmoRos(coz)
    coz_ros.run()


if __name__ == '__main__':
    rospy.init_node('cozmo_driver')
    try:
        cozmo.connect(cozmo_app)
    except cozmo.ConnectionError as e:
        sys.exit('A connection error occurred: {}'.format(e))