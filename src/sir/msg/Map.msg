# Map's lower-left corner (world frame)
sir/Position origin

# Flattened row-major occupancy grid
# (FREE = 0, OCCUPIED = 1, UNKNOWN = -1)
int8[] data

# seconds
builtin_interfaces/Time time_of_validity