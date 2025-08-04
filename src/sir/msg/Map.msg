# Map's upper-left corner (world frame)
sir/Position origin

# Flattened row-major occupancy grid
# [0,0] is top left
# [n,n] is bottom right
# TODO: add costmap levles
# (FREE = 0, OCCUPIED = 1, UNKNOWN = -1)
int8[] data

# seconds
builtin_interfaces/Time time_of_validity