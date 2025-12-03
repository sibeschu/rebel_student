# IGUS Student Helper API

1. Launch `move_group` and the robot simulation as usual.
2. Run `ros2 run igus_student student_workflow` for the minimal demo, or import
   `igus_student.student_api` in your own scripts.
3. Typical usage:
   ```python
   from igus_student import student_api

   robot = student_api.init()
   student_api.move_home()
   student_api.move_to(0.45, 0.2, 0.3, roll=3.14, pitch=0.0, yaw=0.0)
   robot.wait_until_still()
   student_api.shutdown()
   ```
4. Advanced users can instantiate `student_api.StudentRobot` directly and call
   `move_cartesian` / `move_named_pose` for finer control.
