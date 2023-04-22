# a simple way to valid sfm-model

1. disable ego_planner

2. enable sfm_planner

3. publish quadrotor_msgs::PositionCommand with update x, y, z to /position_cmd

4. centerlized control first: all position known, all controllable...

5. global plan first: no feedback from environment...
