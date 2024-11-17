# PID_Controller
_ For file C can take to use in to control Microtrol Microcontroller such that STM32 on motion Robot.
_ For file Python can use simulation and control robot on real with PC .
## Flow to use Simulation:
_ Install ROS2 foxy 
_ Setup file in setup.py in your_package
_ Connect PS4 with PC
_ Run : ros2 run joy joy_node
_ Run : ros2 run your_package ps4_control
_ Run : ros2 run your_package mecanum_pidV1
