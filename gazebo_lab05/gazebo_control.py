import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import numpy as np

class Gazebo_Control(Node):
    def __init__(self):
        super().__init__('gazebo_controller')
        self.get_logger().info("Controller")
        self.init_goal()
        self.init_subscriber()
        self.init_variables()
        self.init_publisher()
        #ros2 topic pub /turtle1/goal geometry_msgs/msg/Pose2D "{x: 7.0, y: 7.0, theta: 0.0}"

    def init_variables(self):
        self.v_max = 0.3
        self.k = 0.5
        self.destino = 0
        return


    def init_goal(self):
        self.goal_publisher = self.create_publisher(Pose2D, '/goal', 10)
        self.timer = self.create_timer(0.5, self.send_goal)
        return

    def send_goal(self):
        msg = Pose2D()
        destino_x = [-1.0, -0.4,0.2,0.6,2.0]
        destino_y = [-0.5, 0.4,0.4,1.4,0.5]

        if self.destino == 0:
            msg.x = destino_x[0]
            msg.y = destino_y[0]
            msg.theta = 0.0
            self.goal_publisher.publish(msg)

        elif self.destino == 1:
            msg.x = destino_x[1]
            msg.y = destino_y[1]
            msg.theta = 0.0
            self.goal_publisher.publish(msg)

        elif self.destino == 2:
            msg.x = destino_x[2]
            msg.y = destino_y[2]
            msg.theta = 0.0
            self.goal_publisher.publish(msg)
        
        elif self.destino == 3:
            msg.x = destino_x[3]
            msg.y = destino_y[3]
            msg.theta = 0.0
            self.goal_publisher.publish(msg)
            
        elif self.destino == 4:
            msg.x = destino_x[4]
            msg.y = destino_y[4]
            msg.theta = 0.0
            self.goal_publisher.publish(msg)
        else:
            pass
    def init_subscriber(self):
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_subscriber = self.create_subscription(Pose2D,'/goal',self.goal_callback,10)
        return
   
    def odom_callback(self, odom: Odometry):
       
        self.posicao_atual_x = odom.pose.pose.position.x
        self.posicao_atual_y = odom.pose.pose.position.y
        self.posicao_atual_theta_1 = odom.pose.pose.orientation.z
        self.posicao_atual_theta_2 = odom.pose.pose.orientation.w
        self.aux1 = 2 * ((self.posicao_atual_theta_1*self.posicao_atual_theta_2) + 0)
        self.aux2 = 1 - (2*(0+(self.posicao_atual_theta_1*self.posicao_atual_theta_1)))
        self.posicao_atual_theta = math.atan2(self.aux1,self.aux2)
        #print(self.posicao_atual_theta)
        #print(self.posicao_atual_theta_1)
        #print(self.posicao_atual_theta_2)
        #print(self.posicao_atual_y)
                
    
    def goal_callback(self, goal: Pose2D):
        self.goal_x = goal.x
        self.goal_y = goal.y
        self.goal_theta = goal.theta
        
    
    
    
    
    def init_publisher(self):
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.send_cmd_vel)
        return

    def send_cmd_vel(self):
        try:
            self.x = self.goal_x - self.posicao_atual_x
            self.y = self.goal_y - self.posicao_atual_y
            if -0.15 < self.x < 0.15 and -0.15 < self.y < 0.15:
                self.destino = self.destino + 1
            if self.destino <= 4:
                print('Erro X: ' ,self.x, 'Erro Y: ', self.y, 'Comando: ', self.destino + 1)
                #print(self.destino)
                self.ro = math.sqrt((self.x)**2+ (self.y)**2)
                self.alpha = math.atan2(self.y,self.x) - self.posicao_atual_theta
                self.v_linear = self.v_max*np.tanh(self.ro)
                self.omega = self.k * self.alpha
                
                msg = Twist()
                msg.linear.x = self.v_linear
                msg.angular.z = self.omega
                self.cmd_vel_publisher.publish(msg)
            else:
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.cmd_vel_publisher.publish(msg)
                print('Objetivo Atingido')
        except AttributeError:
            print('Aguarde')

       
    


def main(args=None):
    #print('Hi from turtle_control_mpl.')
    rclpy.init(args=args)
    node = Gazebo_Control()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()