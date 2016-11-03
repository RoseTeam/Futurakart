#/usr/bin/env python

# Based on:
# http://mathworld.wolfram.com/MomentofInertia.html


def get_cube_inertia_matrix(mass, x, y, z):
    """Given mass and dimensions of a cube return intertia matrix.
    :return: ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz
    From https://www.wolframalpha.com/input/?i=moment+of+inertia+cube"""
    ixx = (1.0 / 12.0) * (y**2 + z**2) * mass
    iyy = (1.0 / 12.0) * (x**2 + z**2) * mass
    izz = (1.0 / 12.0) * (x**2 + y**2) * mass
    ixy = 0.0
    ixz = 0.0
    iyz = 0.0
    return [[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]]


def get_sphere_inertia_matrix(mass, radius):
    """Given mass and radius of a sphere return inertia matrix.
    :return: ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz
    From https://www.wolframalpha.com/input/?i=inertia+matrix+sphere
     """
    ixx = iyy = izz = (2.0 / 5.0) * radius**2 * mass
    ixy = 0.0
    ixz = 0.0
    iyz = 0.0
    return [[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]]


def get_cylinder_inertia_matrix(mass, radius, height):
    """Given mass and radius and height of a cylinder return inertia matrix.
    :return: ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz
    https://www.wolframalpha.com/input/?i=inertia+matrix+cylinder&rawformassumption=%7B%22C%22,+%22cylinder%22%7D+-%3E+%7B%22Solid%22%7D
     """
    ixx = (1.0 / 12.0) * (3.0 * radius**2 + height**2) * mass
    iyy = (1.0 / 12.0) * (3.0 * radius**2 + height**2) * mass
    izz = (1.0 / 12.0) * radius**2 * mass
    ixy = 0.0
    ixz = 0.0
    iyz = 0.0
    return [[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]]

if __name__ == '__main__':

    base_lenght = 1.475
    base_width = 0.84
    base_height = 0.02
    base_mass = 5.0
    print "Inertia martix for chassis_link:"
    print get_cube_inertia_matrix(base_mass, base_lenght, base_width, base_height)


    rear_track_mass = 0.3
    rear_track_radius = 0.005
    rear_track_length = 0.51
    print "Inertia martix for rear track link:"
    print get_cylinder_inertia_matrix(rear_track_mass, rear_track_radius, rear_track_length)

    steering_arm_mass = 0.1
    steering_arm_radius = 0.005
    steering_arm_length = 0.12
    print "Inertia martix for steering arm link:"
    print get_cylinder_inertia_matrix(steering_arm_mass, steering_arm_radius, steering_arm_length)

    wheel_mass = 1.0
    wheel_radius = 0.125
    wheel_width = 0.085
    print "Inertia martix for wheel_link:"
    print get_cylinder_inertia_matrix(wheel_mass, wheel_radius, wheel_width)





