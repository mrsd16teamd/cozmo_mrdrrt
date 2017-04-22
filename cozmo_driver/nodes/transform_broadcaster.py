import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import (Twist, TransformStamped, PoseStamped)

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
