### Script to transform Euler's Roll/Pitch/Yaw angles in degrees to Quaternions

# Python
import sys
import math

# ROS
from tf.transformations import quaternion_from_euler, euler_from_quaternion


def transform(roll, pitch, yaw):
    q = quaternion_from_euler(roll, pitch, yaw)
    return q


def inverse_transform(q0, q1, q2, q3):
    a = euler_from_quaternion([q0, q1, q2, q3])
    return a


if __name__ == "__main__":

    args = sys.argv[1:]
    if len(args) == 3:
        r = float(args[0]) * math.pi/180.0
        p = float(args[1]) * math.pi/180.0
        y = float(args[2]) * math.pi/180.0
        q = transform(r, p, y)
        print q[0], q[1], q[2], q[3]
    elif len(args) == 4:
        q0 = float(args[0])
        q1 = float(args[1])
        q2 = float(args[2])
        q3 = float(args[3])
        a = inverse_transform(q0, q1, q2, q3)
        print a[0] * 180.0/math.pi, a[1] * 180.0/math.pi, a[2] * 180.0/math.pi
    else:
        print "Usage : angles to quat: transform_angles.py roll pitch yaw"
        print "Usage : quat to angles : transform_angles.py q0 q1 q2 q3"

