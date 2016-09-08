### Script to compute inverse transform from one frame to another

# Python
import sys
import math

# ROS
from tf.transformations import compose_matrix, decompose_matrix, inverse_matrix

# Project
from transformAngles import transform

if __name__ == "__main__":

    args=sys.argv[1:]
    if len(args) == 6:
        x = float(args[0])
        y = float(args[1])
        z = float(args[2])
        r = float(args[3]) * math.pi/180.0
        p = float(args[4]) * math.pi/180.0
        yw = float(args[5]) * math.pi/180.0

        M = compose_matrix(angles=[r, p, yw], translate=[x, y, z])
        M = inverse_matrix(M)
        _, _, angles, trans, _ = decompose_matrix(M)

        q = transform(angles[0], angles[1], angles[2])

        print trans[0], trans[1], trans[2], q[0], q[1], q[2], q[3]
        print trans[0], trans[1], trans[2], angles[0]* 180.0/math.pi, angles[1]* 180.0/math.pi, angles[2]* 180.0/math.pi

    else:
        print "Usage : transform_frames.py x y z rollDeg pitchDeg yawDeg "
        print "         -> x' y' z' q0' q1' q2' q3'"
        print "         -> x' y' z' rollDeg' pitchDeg' yawDeg'"

