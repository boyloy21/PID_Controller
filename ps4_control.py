import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String, Int8, Float32
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

def map(Input, min_input, max_input, min_output, max_output):
    value = ((Input - min_input)*(max_output-min_output)/(max_input - min_input) + min_output)
    return value
class PS4Control(Node):
    def __init__(self):
        super().__init__('PS4_Node')
        self.pub_timer = 0.01
        self.subscribe_joy = self.create_subscription(
            Joy, "joy", self.subscribe_callback, 10)
        self.input_controls = self.create_publisher(
            Twist, "/manual", 10)
        # self.msg_timer = self.create_timer(self.pub_timer, self.pub_callback)
        self.mode_pub = self.create_publisher(String, "/mode", 10)
        self.goal_pub = self.create_publisher(String, "/goal", 10)
        
        self.axes_list = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.button_list = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.Vx = 0.0
        self.Vy = 0.0
        self.Omega = 0.0
        self.Speed = 0.0
        self.SpeedAngle = 0.0

    def subscribe_callback(self,joy):
        self.axes_list = joy.axes
        self.button_list = joy.buttons
        manual = Twist()
        mode = String()
        goal = String()
        if ((self.button_list[4] == 1)):
            mode.data = "manual"
            self.mode_pub.publish(mode)
            self.get_logger().info('Mode : "%s"' % (mode.data))
        if ((self.button_list[5] == 1)):
            mode.data = "auto"
            self.mode_pub.publish(mode)
            self.get_logger().info('Mode : "%s"' % (mode.data))
        if ((self.button_list[0] == 1)):
            goal.data = "goal1"
            self.goal_pub.publish(goal)
            self.get_logger().info('Goal : "%s"' % (goal.data))
        if ((self.button_list[1] == 1)):
            goal.data = "goal2"
            self.goal_pub.publish(goal)
            self.get_logger().info('Goal : "%s"' % (goal.data))
        if ((self.button_list[2] == 1)):
            goal.data = "goal3"
            self.goal_pub.publish(goal)
            self.get_logger().info('Goal : "%s"' % (goal.data))
        if ((self.button_list[3] == 1)):
            goal.data = "goal4"
            self.goal_pub.publish(goal)
            self.get_logger().info('Goal : "%s"' % (goal.data))
        if ((self.button_list[11] == 1)):
            goal.data = "back1"
            self.goal_pub.publish(goal)
            self.get_logger().info('Goal : "%s"' % (goal.data))
        if (self.axes_list[7] == 1.0):
            self.Speed += 0.05
        elif (self.axes_list[7] == -1.0):
            self.Speed -= 0.05
        elif (self.axes_list[7] == 0.0 ):
            self.Speed = self.Speed
        if (self.axes_list[6] == 1.0):
            self.SpeedAngle += 0.05
        elif (self.axes_list[6] == -1.0):
            self.SpeedAngle -= 0.05
        elif (self.axes_list[6] == 0.0):
            self.SpeedAngle = self.SpeedAngle
        if (self.axes_list[1] != 0.0):
            self.Vx = (self.axes_list[1])*(1.0 + self.Speed)
            if ((self.Vx < 0.6 and self.Vx > -0.6) or (self.Vx > 2.0 or self.Vx < -2.0)):
                self.Vx = 0.0
            manual.linear.x = self.Vx
            self.input_controls.publish(manual)
            self.get_logger().info('Manual control - Linear Velocity X : %f, Linear Velocity Y : %f, Angular Velocity Z : %f' % (self.Vx, self.Vy,self.Omega))
        if (self.axes_list[0] !=0.0 ):
            self.Vy = (self.axes_list[0])*(1.0 + self.Speed)
            if ((self.Vy < 0.6 and self.Vy > -0.6) or (self.Vy > 2.0 or self.Vy < -2.0)):
                self.Vy = 0.0
            manual.linear.y = self.Vy
            self.input_controls.publish(manual)
            self.get_logger().info('Manual control - Linear Velocity X : %f, Linear Velocity Y : %f, Angular Velocity Z : %f' % (self.Vx, self.Vy,self.Omega))
        if (self.axes_list[3] !=0.0):
            self.Omega = (self.axes_list[3])*(1.5 + self.SpeedAngle)
            if ((self.Omega < 0.6 and self.Omega > -0.6) or (self.Omega > 3.14 or self.Omega < -3.14)):
                self.Omega = 0.0
            manual.angular.z = self.Omega
            self.input_controls.publish(manual)
            self.get_logger().info('Manual control - Linear Velocity X : %f, Linear Velocity Y : %f, Angular Velocity Z : %f' % (self.Vx, self.Vy,self.Omega))
        
def main(args=None):
    rclpy.init(args=args)
    PS4 = PS4Control()
    rclpy.spin(PS4)
    PS4.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()