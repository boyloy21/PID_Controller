# PID_Controller
_ For file C can take to use in to control Microtrol Microcontroller such that STM32 on motion Robot.<br>
_ For file Python can use simulation and control robot on real with PC .
## Flow to use Simulation:
1. Install ROS2 foxy
2. Setup file in setup.py in your_package
3. Connect PS4 with PC
4. Run : ros2 run joy joy_node
5. Run : ros2 run your_package ps4_control
6. Run : ros2 run your_package pid_simulation
