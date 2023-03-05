# Author: Addison Sears-Collins
# Date: March 19, 2021
# ROS Version: ROS 2 Foxy Fitzroy

############## IMPORT LIBRARIES #################
# Python enumeration library
import enum

# Python math library
import math

# Scientific computing library
import numpy as np

# ROS client library for Python
import rclpy

# Enables pauses in the execution of code
from time import sleep

# Used to create nodes
from rclpy.node import Node

# Enables the use of the string message type
from std_msgs.msg import String

# Twist is linear and angular velocity
from geometry_msgs.msg import Twist

# Handles LaserScan messages to sense distance to obstacles (i.e. walls)
from sensor_msgs.msg import LaserScan

# Handle Pose messages
from geometry_msgs.msg import Pose

# Handle float64 arrays
from std_msgs.msg import Float64MultiArray

# Handles quality of service for LaserScan data
from rclpy.qos import qos_profile_sensor_data

# Position, orientation, linear velocity, angular velocity
from nav_msgs.msg import Odometry


PATHS = [
    [
        [1, 1],
        [1.5211734668953483, 1.85345077034606],
        [2.1524890790349103, 2.628976784009593],
        [2.1951536610401527, 3.628066236182079],
        [2.2634960650884683, 4.625728160788253],
        [1.9108717964891921, 5.561493155428908],
        [1.6399469946507694, 6.524093670569437],
        [2.610706858833114, 6.764146344148958],
        [3.4733430489358867, 6.913776192182645],
        [4.063786095779166, 7.720855499583365],
        [4.545338852916487, 8.408445889431924],
        [3.890700517489327, 8.977692087783634],
        [3.2806649568233857, 9.461279293238771],
        [4.280658760121673, 9.46479971438221],
        [5.279594415672584, 9.418674275966175],
        [6.2619189297302125, 9.231488408226532],
        [6.739553963687831, 8.352930058237014],
        [7.610199306178251, 7.861018798353956],
        [7.269025146972788, 6.921018695646278],
        [8.1246712387962, 6.403457331605914],
        [9.006578303847169, 5.932034030725286],
        [9.965663217436362, 6.21515260751885],
        [10.299704592962263, 5.272594195834586],
        [10.725658027023526, 4.3678491090773175],
        [10.559616829668256, 3.3817302923108854],
        [11.085693811007042, 2.5312933400883995],
        [11.0, 3.0]
    ],
    [
        [11.0, 3.0],
        [11.085693811007042, 2.5312933400883995],
        [10.559616829668256, 3.3817302923108854],
        [10.725658027023526, 4.3678491090773175],
        [10.299704592962263, 5.272594195834586],
        [9.965663217436362, 6.21515260751885],
        [9.006578303847169, 5.932034030725286],
        [8.1246712387962, 6.403457331605914],
        [7.269025146972788, 6.921018695646278],
        [7.610199306178251, 7.861018798353956],
        [6.739553963687831, 8.352930058237014],
        [6.2619189297302125, 9.231488408226532],
        [5.279594415672584, 9.418674275966175],
        [4.280658760121673, 9.46479971438221],
        [3.2806649568233857, 9.461279293238771],
        [3.890700517489327, 8.977692087783634],
        [4.545338852916487, 8.408445889431924],
        [4.063786095779166, 7.720855499583365],
        [3.4733430489358867, 6.913776192182645],
        [2.610706858833114, 6.764146344148958],
        [1.6399469946507694, 6.524093670569437],
        [1.9108717964891921, 5.561493155428908],
        [2.2634960650884683, 4.625728160788253],
        [2.1951536610401527, 3.628066236182079],
        [2.1524890790349103, 2.628976784009593],
        [1.5211734668953483, 1.85345077034606],
        [1, 1]
    ]
]


class State(enum.Enum):
    PATH_SELECT = 0,
    WAYPOINT_SELECT = 1,
    GRASPING = 2
    ANGLE_CONTROL = 3,
    LINEAR_CONTROL = 4


class PID:
    """
    Discrete PID control
    """

    def __init__(self, P=0.0, I=0.0, D=0.0, Derivator=0, Integrator=0, Integrator_max=10, Integrator_min=-10, cmd_max = None, cmd_min = None):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.Derivator = Derivator
        self.Integrator = Integrator
        self.Integrator_max = Integrator_max
        self.Integrator_min = Integrator_min
        self.cmd_max = None
        self.cmd_min = None
        self.set_point = 0.0
        self.error = 0.0

    def update(self, current_value):
        self.error = self.set_point - current_value
        if self.error > math.pi:  # specific design for circular situation
            self.error = self.error - 2*math.pi
        elif self.error < -math.pi:
            self.error = self.error + 2*math.pi
        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * (self.error - self.Derivator)
        self.Derivator = self.error
        self.Integrator = self.Integrator + self.error
        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min
        self.I_value = self.Integrator * self.Ki
        PID = self.P_value + self.I_value + self.D_value
        if self.cmd_max is not None and PID > self.cmd_max:
            PID = self.cmd_max
        elif self.cmd_min is not None and PID < self.cmd_min:
            PID = self.cmd_min
        return PID

    def setPoint(self, set_point):
        self.set_point = set_point
        self.Derivator = 0
        self.Integrator = 0

    def setPID(self, set_P=0.0, set_I=0.0, set_D=0.0):
        self.Kp = set_P
        self.Ki = set_I
        self.Kd = set_D
    
    def setCmdLimits(self, cmd_max, cmd_min):
        self.cmd_max = cmd_max
        self.cmd_min = cmd_min

    def reset(self):
        self.error = 0
        self.Derivator = 0
        self.Integrator = 0


class Controller(Node):
    """
    Create a Controller class, which is a subclass of the Node 
    class for ROS2.
    """

    def __init__(self):
        super().__init__('controller')
        self.state = State.PATH_SELECT
        self.pid_theta = PID(1, 0, 0, cmd_max=20, cmd_min=-20)
        self.paths = PATHS
        self.waypoints = []
        self.target = None
        self.theta = 0.0
        self.vel = Twist()

        self.vel_pub = self.create_publisher(Twist, '/pr2/cmd_vel', 10)

        self.odom_sub = self.create_subscription(Odometry, '/pr2/odom',
                                                 self.odom_callback, 10)
    
    # Should be called when target or pose updated          
    def reroute(self, target):
        self.diff_x = target[0] - self.x
        self.diff_y = target[1] - self.y
        vector = np.array([self.diff_x, self.diff_y])
        self.linear = math.sqrt(self.diff_x * self.diff_x + self.diff_y * self.diff_y)
        self.direction_vector = vector / self.linear  # normalization
        self.theta = math.atan2(self.diff_y, self.diff_x)

    def odom_callback(self, msg):
        """
        Receive the odometry information containing the position and orientation
        of the robot in the global reference frame. 
        The position is x, y, z.
        The orientation is a x,y,z,w quaternion. 
        """
        roll, pitch, yaw = self.euler_from_quaternion(
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
        self.x, self.y, self.yaw = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]
        if self.target is not None:
            self.reroute(self.target)
        # print(f"Pose: {msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f}, {yaw:.2f}")

        while True:
            print(f"State: {self.state}, Pose: {self.x:.2f}, {self.y:.2f}, {self.yaw:.2f} ==>> Target: {self.target}")
            match self.state:
                case State.PATH_SELECT:
                    if len(self.paths) == 0:
                        print("ALL MOTIONS ARE COMPLETED")
                        return
                    else:
                        self.waypoints = self.paths.pop(0)
                        self.waypoints.pop(0)
                        self.state = State.WAYPOINT_SELECT
                        
                case State.WAYPOINT_SELECT:
                    if len(self.waypoints) == 1:
                        self.state = State.GRASPING
                    else:
                        self.target = self.waypoints.pop(0)
                        self.reroute(self.target)
                        self.pid_theta.setPoint(self.theta)
                        self.state = State.ANGLE_CONTROL
                        
                case State.GRASPING:
                    print("Some grasping action: ", self.waypoints.pop())
                    self.state = State.PATH_SELECT
                    return
                
                case State.ANGLE_CONTROL:
                    if abs(self.theta - self.yaw) < 0.02:
                        self.state = State.LINEAR_CONTROL
                    else:
                        self.pid_theta.setPID(10, 1, 0)
                        angular = self.pid_theta.update(self.yaw)
                        self.vel.linear.x = 0.0
                        self.vel.angular.z = angular
                        print(f"Diff: {self.theta - self.yaw:.2f} --> Command: {self.vel.angular}\n\n\n")
                        self.vel_pub.publish(self.vel)
                        return
                        
                case State.LINEAR_CONTROL:
                    self.pid_theta.reset()
                    if abs(self.theta - self.yaw) > 0.1:
                        self.state = State.ANGLE_CONTROL
                    elif abs(self.linear) < 0.01:
                        self.state = State.WAYPOINT_SELECT
                    else:
                        self.pid_theta.setPID(1, 0.02, 0.5)
                        angular = self.pid_theta.update(self.yaw)
                        if abs(self.linear) > 1:
                            self.linear = self.linear / abs(self.linear) * 1
                        self.vel.linear.x = self.linear * 3
                        self.vel.angular.z = angular
                        print(f"          Diff: {self.theta:.2f} --> Command: {self.vel}\n\n\n")
                        self.vel_pub.publish(self.vel)
                        return

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z  # in radians
    

def main(args=None):
    rclpy.init(args=args)
    controller = Controller()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
