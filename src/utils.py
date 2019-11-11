import tf.transformations

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

def angle_to_rosquaternion(angle):
    """Convert an angle in radians into a quaternion _message_."""
    return Quaternion(*tf.transformations.quaternion_from_euler(0, 0, angle))


def rosquaternion_to_angle(q):
    """Convert a quaternion _message_ into an angle in radians.
    The angle represents the yaw.
    This is not just the z component of the quaternion."""
    x, y, z, w = q.x, q.y, q.z, q.w
    _, _, yaw = tf.transformations.euler_from_quaternion((x, y, z, w))
    return yaw


def rospose_to_posetup(posemsg, map):
    """Convert global pose to map coordinates"""
    x = int((posemsg.position.x - map.origin.position.x)/map.resolution)
    y = int((posemsg.position.y - map.origin.position.y)/map.resolution)
    th = rosquaternion_to_angle(posemsg.orientation)
    return x, y, th


def posetup_to_rospose(posetup, map):
    """Convert map coordinates to ros pose"""
    pose = Pose()
    x = posetup[0]*map.resolution + map.origin.position.x
    y = posetup[1]*map.resolution + map.origin.position.y
    pose.position = Point(x, y, 0)
    pose.orientation = angle_to_rosquaternion(posetup[2])
    return pose
