from math import pi

PLANNING_GROUP = "igus_rebel_arm"
END_EFFECTOR_LINK = "ee_link"
REFERENCE_FRAME = "base_link"

POSITION_TOLERANCE = 0.01  # m
ORIENTATION_TOLERANCE = 0.01 # radians
VELOCITY_THRESHOLD = 0.05 # for robot moving check
MAX_VEL_FACTOR = 0.5
MAX_ACCEL_FACTOR = 0.5

HOME_POSITION = (0.4, 0.0, 0.3) # x, y, z
HOME_ORIENTATION = (pi, 0.0, 0.0)  # roll, pitch, yaw
