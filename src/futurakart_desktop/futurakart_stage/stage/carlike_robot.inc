define laser ranger
(
  sensor
  (
    range_max 4.0
    fov 57.0
    samples 320
  )
  # generic model properties
  color "black"
  size [ 0.06 0.15 0.03 ]
)

#
# Robot model:

define carlike_robot position
(
  pose [ 0.0 0.0 0.0 0.0 ]

  odom_error [0.03 0.03 999999 999999 999999 0.02]


  # Dimensions taken from dimensions.urdf
  #
  # base_length = 1.475
  # base_width = 0.84
  #
  size [ 1.475 0.84 0.5 ]
  # correct center of rotation:
  origin [ 0.0 0.0 0.0 0.0]
  gui_nose 1
  color "red"

  # kinematics
  drive "car"
  wheelbase 1.0 # distance between both axles
  
  # spawn sensors
  laser(
    pose [ 0.3 0.0 0.4 0.0 ]
  )
)
