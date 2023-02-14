#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt, pi

PI = pi

class TurtleBot:
    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=True)

        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)

        self.pose = Pose()
        self.rate = rospy.Rate(10)
    
    def update_pose(self, data):
        
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
    def init_pose(self):
        while(self.pose.x == 0.0 ):
            pass
    
    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))
    def linear_vel(self, goal_pose, constant=1.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=6):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)
    
    def rotate(self,angle, clockwise=True):
        #Starts a new node
        #rospy.init_node('robot_rotator', anonymous=True)
        #velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        vel_msg = Twist()
    
        # Receiveing the user's input
        #print("Let's rotate your robot")
        speed = 50
        #angle = float(input("Type your distance (degrees):"))
        
    
        #Converting from angles to radians
        angular_speed = speed*2*PI/360
        relative_angle = angle*2*PI/360
    
        #We wont use linear components
        vel_msg.linear.x=0
        vel_msg.linear.y=0
        vel_msg.linear.z=0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
    
        # Checking if our movement is CW or CCW
        if (clockwise == True):
            vel_msg.angular.z = -abs(angular_speed)
        elif(clockwise == False):
            vel_msg.angular.z = abs(angular_speed)
        # Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_angle = 0
    
        while(current_angle < relative_angle):
            self.velocity_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed*(t1-t0)
            
    
    
        #Forcing our robot to stop
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
    
    def rotate_to_angle(self,angle, clockwise=True):
        
        vel_msg = Twist()
    
        self.init_pose()
        
        vel_msg.linear.x=0
        vel_msg.linear.y=0
        vel_msg.linear.z=0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        angular_speed = 1
        current_angle = self.pose.theta
        
        # Checking if our movement is CW or CCW
        
        if (abs(angle - current_angle)<0.01):
            pass
        else:
            print(abs(angle - current_angle))
            if (abs(angle - current_angle) > 3.1399999):
                try:
                    vel_msg.angular.z = -int(abs(angle)/angle)*abs(angular_speed)
                except:
                    vel_msg.angular.z = -1
            else:
                
                try:
                    vel_msg.angular.z = int(abs(angle)/angle)*abs(angular_speed)
                except:
                    vel_msg.angular.z = 1
            
        
            while(abs(current_angle - angle)>=0.01):
                self.velocity_publisher.publish(vel_msg)
                current_angle = self.pose.theta
                #print(abs(current_angle - angle))
                
            #Forcing our robot to stop
            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)
    
    def goAhead(self, distance):
        vel_msg = Twist()
    
        speed = 2
    
        #We wont use linear components
        vel_msg.linear.x=speed
        vel_msg.linear.y=0
        vel_msg.linear.z=0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        
        t0 = rospy.Time.now().to_sec()
        current_position = 0
        while(current_position <= distance):
            self.velocity_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_position = speed*(t1-t0)
            rospy.loginfo("x: %s y: %s", self.pose.x, self.pose.y)
    
    
        #Forcing our robot to stop
        vel_msg.linear.x = 0
        self.velocity_publisher.publish(vel_msg)

    def go_for_point_from_initial(self, x, y):
        self.init_pose()
        self.goAhead(1)
        self.rotate(90, True)
        
        self.goAhead(self.pose.y - y)
        self.rotate(90, True)
        self.goAhead(self.pose.x - x)
        self.rotate(90, True)
    
    def go_to_goal(self, x, y):
        
        goal_pose = Pose()
        goal_pose.x = x
        goal_pose.y = y
        
        distance_tolerance = 0.1
        
        vel_msg = Twist()
        
        while(self.euclidean_distance(goal_pose) >= distance_tolerance):
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)

            
            self.velocity_publisher.publish(vel_msg)

            
            self.rate.sleep()
            
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
    
    def spiral_turn(self, limit):
        self.init_pose()
        speed = 0.5
        while(self.pose.x <= limit):
            vel_msg = Twist()

            vel_msg.linear.x=speed
            vel_msg.linear.y=0
            vel_msg.linear.z=0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 1
            TB.velocity_publisher.publish(vel_msg)
            
            speed += 0.000001
    
    def clean_up(self, x, y):
        self.init_pose()
        self.go_to_goal(x, y)
        for _ in range(5):
            self.rotate_to_angle(1.57)
            self.goAhead(9)
            self.rotate_to_angle(0)
            self.goAhead(1)
            self.rotate_to_angle(-1.57)
            self.goAhead(9)
            self.rotate_to_angle(0)
            self.goAhead(1)

if __name__ == '__main__':
    try:
        TB = TurtleBot()
        
        TB.clean_up(1, 1)

    except rospy.ROSInterruptException:
        pass
