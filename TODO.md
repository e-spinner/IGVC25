
# Todo
- Pathing system does not respect or care about theta_z
- Pathing also will crash if start or goal outside of map grid, no error handling, shoudl probably allow it to path out of bounds somewhat.
- Pathing also also does nto respect the actual size of the robot

# Issues
 - [**#1**] Occasionally pather returns nearly empty path, either failed to find path, and did not error, or the reconstruction of path faulted