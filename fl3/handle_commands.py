################################################
# IMPORTS and SETUP
################################################

# math imports
import numpy as np

# ros2 imports
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

# topic imports
from geometry_msgs.msg import PoseArray, PoseStamped, Point, Quaternion
from nav_msgs.msg import Odometry

# reliability imports
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=1)

################################################
# NODES
################################################

class CommNode(Node):

    def __init__(self, test_type):
        super().__init__('rob498_drone_1') 

        ###############
        # SERVICE CALLS

        self.srv_launch = self.create_service(Trigger, 'rob498_drone_1/comm/launch', self.callback_launch)
        self.srv_test = self.create_service(Trigger, 'rob498_drone_1/comm/test', self.callback_test)
        self.srv_land = self.create_service(Trigger, 'rob498_drone_1/comm/land', self.callback_land)
        self.srv_abort = self.create_service(Trigger, 'rob498_drone_1/comm/abort', self.callback_abort)
        self.sub_waypoints = self.create_subscription(PoseArray, 'rob498_drone_1/comm/waypoints', self.callback_waypoints, 10)
        print('services created')

        ################
        # VARIABLE SETUP

        # for the launching, we start ready to set the waypoint
        self.height = 0.1

        # for vision_pose to know where it is
        self.position = Point()
        self.orientation = Quaternion()
        self.timestamp = None
        self.frame_id = "map"

        # for setpoint_vision to know where to go
        self.set_position = Point()
        self.set_orientation = Quaternion()
        self.set_orientation.w = -1.0

        # # inital global variables
        # self.WAYPOINTS = None
        # self.WAYPOINTS_RECEIVED = False
        # self.set_this_waypoint = False
        # self.waypoint_index = 0
        # self.waypoint = None

        # tolerance for grading
        self.tol_radius = 0.1 #0.4

        # inital global variables
        h = 0.5
        self.WAYPOINTS = np.array([[0, 0, h],[0, h, 0],[h, 0, 0],[h, h, h]])
        self.WAYPOINTS_RECEIVED = True
        self.set_this_waypoint = False
        self.waypoint_index = 0
        self.waypoint = None
        self.in_test = False

        ############################
        # SUBSCRIBER/PUBLISHER SETUP

        if test_type == "realsense":
            # Subscriber to RealSense pose data
            self.realsense_subscriber = self.create_subscription(Odometry, '/camera/pose/sample', self.realsense_callback, qos_profile)
            self.get_logger().info('Subscribing to RealSense!')
        else: 
            # Subscriber to Vicon pose data
            self.vicon_subscriber = self.create_subscription(PoseStamped, '/vicon/ROB498_Drone/ROB498_Drone', self.vicon_callback, 1)
            self.get_logger().info('Subscribing to Vicon!')
        
        # Publisher for VisionPose topic
        self.vision_pose_publisher = self.create_publisher(PoseStamped, '/mavros/vision_pose/pose', 1)
        self.get_logger().info('Publishing to VisionPose')

        # Publisher for SetPoint topic
        self.setpoint_publisher = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', qos_profile)
        self.get_logger().info('Publishing to SetPoint')

        # Statement to end the inits
        self.get_logger().info('Nodes All Setup and Started!')
        

    ################################################
    # SERVICE CALLS 
    ################################################

    def callback_launch(self, request, response):
        print('Launch Requested. Your drone should take off.')
        self.set_pose_initial()
        self.set_position.z = self.height
        return response

    def callback_test(self, request, response):
        print('Test Requested. Your drone should perform the required tasks. Recording starts now.')
        self.in_test = True
        return response
        
    def test_loop(self):
        # check if we're got points and are good to go
        # print(self.in_test)
        if self.in_test and self.WAYPOINTS_RECEIVED:
            # loop through the waypoints
            if self.waypoint_index < len(self.WAYPOINTS):
                print(self.waypoint_index, self.waypoint)

                if not self.set_this_waypoint:
                    self.waypoint = self.WAYPOINTS[self.waypoint_index]
                    print(self.waypoint)
                    self.set_waypoint(self.waypoint)
                    self.set_this_waypoint = True

                # changing to the next index when close enough
                if self.close_enough(self.waypoint):
                    self.set_this_waypoint = False
                    self.waypoint_index += 1
                    print('Waypoint Reached:', self.waypoint)

            else:
                self.in_test = False
                print("Done traversing waypoints!")
        else:
            print('No waypoints received. Cannot test.')


    def callback_land(self, request, response):
        print('Land Requested. Your drone should land.')
        self.set_position.z = 0.1
        response.success = True
        response.message = "Success"
        return response

    def callback_abort(self, request, response):
        print('Abort Requested. Your drone should land immediately due to safety considerations.')
        self.set_position.z = 0.0
        response.success = True
        response.message = "Success"
        return response


    ################################################
    # SERVICE CALLS 
    ################################################
    
    def realsense_callback(self, msg):
        # get the info
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation
        self.timestamp = self.get_clock().now().to_msg()
        # frame conversion
        self.orientation.x *= -1
        self.orientation.y *= -1
        self.orientation.z *= -1
        self.orientation.w *= -1
        # print statements to know its running
        # print(f"Position: x={self.position.x}, y={self.position.y}, z={self.position.z}")
        # print(f"Orientation: x={self.orientation.x}, y={self.orientation.y}, z={self.orientation.z}, w={self.orientation.w}")
        # print(f"Timestamp: {self.timestamp.sec}.{self.timestamp.nanosec}")
        # print(f"Frame ID: {self.frame_id}")
        # WRITE BOTH IMMEDIATELY
        self.send_vision_pose()
        self.send_setpoint()
        self.test_loop()

    def vicon_callback(self, msg):
        # get the info
        self.position = msg.pose.position
        self.orientation = msg.pose.orientation
        self.timestamp = self.get_clock().now().to_msg()
        # frame conversion
        self.orientation.x *= -1
        self.orientation.y *= -1
        self.orientation.z *= -1
        self.orientation.w *= -1
        # print statements to know its running
        # print(f"Position: x={self.position.x}, y={self.position.y}, z={self.position.z}")
        # print(f"Orientation: x={self.orientation.x}, y={self.orientation.y}, z={self.orientation.z}, w={self.orientation.w}")
        # print(f"Timestamp: {self.timestamp.sec}.{self.timestamp.nanosec}")
        # print(f"Frame ID: {self.frame_id}")
        # WRITE BOTH IMMEDIATELY
        self.send_vision_pose()
        self.send_setpoint()

    def send_vision_pose(self):
        # Create a new PoseStamped message to publish to vision_pose topic
        vision_pose_msg = PoseStamped()
        vision_pose_msg.header.stamp = self.timestamp
        vision_pose_msg.header.frame_id = self.frame_id
        vision_pose_msg.pose.position = self.position
        vision_pose_msg.pose.orientation = self.orientation
        # Publish the message to the /mavros/vision_pose/pose topic
        self.vision_pose_publisher.publish(vision_pose_msg)

    def send_setpoint(self):
        # Create a new PoseStamped message to publish to setpoint topic
        setpoint_msg = PoseStamped()
        setpoint_msg.header.stamp = self.timestamp
        setpoint_msg.header.frame_id = self.frame_id
        setpoint_msg.pose.position = self.set_position
        setpoint_msg.pose.orientation = self.set_orientation
        # Publish the message to the /mavros/setpoint_position/local topic
        self.setpoint_publisher.publish(setpoint_msg)

    def callback_waypoints(self, msg):
        if self.WAYPOINTS_RECEIVED:
            return
        print('Waypoints Received')
        self.WAYPOINTS_RECEIVED = True
        self.WAYPOINTS = np.empty((0,3))
        for pose in msg.poses:
            pos = np.array([pose.position.x, pose.position.y, pose.position.z])
            self.WAYPOINTS = np.vstack((self.WAYPOINTS, pos))


    ################################################
    # HELPER FUNCTIONS
    ################################################

    def set_pose_initial(self):
        # Put the current position into maintained position
        self.set_position.x = 0.0
        self.set_position.y = 0.0
        self.set_position.z = 0.0
        self.set_orientation.x = 0.0
        self.set_orientation.y = 0.0
        self.set_orientation.z = 0.0
        self.set_orientation.w = -1.0

    def set_waypoint(self, waypoint):
        # Put the current position into maintained position
        self.set_position.x = waypoint[0]
        self.set_position.y = waypoint[1]
        self.set_position.z = waypoint[2]
        self.set_orientation.x = 0.0
        self.set_orientation.y = 0.0
        self.set_orientation.z = 0.0
        self.set_orientation.w = -1.0

    def close_enough(self, waypoint):
        # extract the current and waypoint positions
        curr_x, curr_y, curr_z = self.position.x, self.position.y, self.position.z
        w_x, w_y, w_z = waypoint[0], waypoint[1], waypoint[2]
        # check if the drone is close enough to the waypoint
        # using euclidean distance
        if np.sqrt((curr_x - w_x)**2 + (curr_y - w_y)**2 + (curr_z - w_z)**2) < self.tol_radius:
            return True
        # otherwise, not close enough yet
        return False


################################################
# MAIN
################################################
        
def main(args=None):

    # global and local variables
    # global STATE, WAYPOINTS, WAYPOINTS_RECEIVED, set_this_waypoint, waypoint_index
    test_type = "realsense"

    # setup the node the run
    rclpy.init(args=args)
    comm_node = CommNode(test_type)
    print("FLIGHT TEST #3 LET'S GO!")
    rclpy.spin(comm_node) 
    comm_node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()












# ################################################
# # IMPORTS and SETUP
# ################################################

# # math imports
# import numpy as np

# # ros2 imports
# import rclpy
# from rclpy.node import Node
# from std_srvs.srv import Trigger

# # topic imports
# from geometry_msgs.msg import PoseArray, PoseStamped, Point, Quaternion
# from nav_msgs.msg import Odometry

# # reliability imports
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy
# qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=1)

# # inital global variables
# STATE = 'Init'
# WAYPOINTS = None
# WAYPOINTS_RECEIVED = False
# set_this_waypoint = False
# waypoint_index = 0

# ################################################
# # NODES
# ################################################

# class CommNode(Node):

#     def __init__(self, test_type):
#         super().__init__('rob498_drone_1') 

#         ###############
#         # SERVICE CALLS

#         self.srv_launch = self.create_service(Trigger, 'rob498_drone_1/comm/launch', self.callback_launch)
#         self.srv_test = self.create_service(Trigger, 'rob498_drone_1/comm/test', self.callback_test)
#         self.srv_land = self.create_service(Trigger, 'rob498_drone_1/comm/land', self.callback_land)
#         self.srv_abort = self.create_service(Trigger, 'rob498_drone_1/comm/abort', self.callback_abort)
#         self.sub_waypoints = self.create_subscription(PoseArray, 'rob498_drone_1/comm/waypoints', self.callback_waypoints, 10)
#         print('services created')

#         ################
#         # VARIABLE SETUP

#         # for the launching, we start ready to set the waypoint
#         self.height = 1.5

#         # tolerance for grading
#         self.tol_radius = 0.3 #0.4

#         # for vision_pose to know where it is
#         self.position = Point()
#         self.orientation = Quaternion()
#         self.timestamp = None
#         self.frame_id = "map"

#         # for setpoint_vision to know where to go
#         self.set_position = Point()
#         self.set_orientation = Quaternion()
#         self.set_orientation.w = -1.0

#         ############################
#         # SUBSCRIBER/PUBLISHER SETUP

#         if test_type == "realsense":
#             # Subscriber to RealSense pose data
#             self.realsense_subscriber = self.create_subscription(Odometry, '/camera/pose/sample', self.realsense_callback, qos_profile)
#             self.get_logger().info('Subscribing to RealSense!')
#         else: 
#             # Subscriber to Vicon pose data
#             self.vicon_subscriber = self.create_subscription(PoseStamped, '/vicon/ROB498_Drone/ROB498_Drone', self.vicon_callback, 1)
#             self.get_logger().info('Subscribing to Vicon!')
        
#         # Publisher for VisionPose topic
#         self.vision_pose_publisher = self.create_publisher(PoseStamped, '/mavros/vision_pose/pose', 1)
#         self.get_logger().info('Publishing to VisionPose')

#         # Publisher for SetPoint topic
#         self.setpoint_publisher = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', qos_profile)
#         self.get_logger().info('Publishing to SetPoint')

#         # Statement to end the inits
#         self.get_logger().info('Nodes All Setup and Started!')
        

#     ################################################
#     # SERVICE CALLS 
#     ################################################

#     def callback_launch(self, request, response):
#         # global STATE
#         # STATE = 'Launch'
#         print('Launch Requested. Your drone should take off.')
#         self.set_pose_initial()
#         self.set_position.z = self.height
#         return response

#     def callback_test(self, request, response):
#         global STATE
#         STATE = 'Test'
#         print('Test Requested. Your drone should perform the required tasks. Recording starts now.')
#         return response

#     def callback_land(self, request, response):
#         # global STATE
#         # STATE = 'Land'
#         print('Land Requested. Your drone should land.')
#         self.set_position.z = 0.1
#         return response

#     def callback_abort(self, request, response):
#         # global STATE
#         # STATE = 'Abort'
#         print('Abort Requested. Your drone should land immediately due to safety considerations.')
#         self.set_position.z = 0.0
#         response.success = True
#         response.message = "Success"
#         return response


#     ################################################
#     # SERVICE CALLS 
#     ################################################
    
#     def realsense_callback(self, msg):
#         # get the info
#         self.position = msg.pose.pose.position
#         self.orientation = msg.pose.pose.orientation
#         self.timestamp = self.get_clock().now().to_msg()
#         # frame conversion
#         self.orientation.x *= -1
#         self.orientation.y *= -1
#         self.orientation.z *= -1
#         self.orientation.w *= -1
#         # print statements to know its running
#         print(f"Position: x={self.position.x}, y={self.position.y}, z={self.position.z}")
#         print(f"Orientation: x={self.orientation.x}, y={self.orientation.y}, z={self.orientation.z}, w={self.orientation.w}")
#         print(f"Timestamp: {self.timestamp.sec}.{self.timestamp.nanosec}")
#         print(f"Frame ID: {self.frame_id}")
#         # WRITE BOTH IMMEDIATELY
#         self.send_vision_pose()
#         self.send_setpoint()

#     def vicon_callback(self, msg):
#         # get the info
#         self.position = msg.pose.position
#         self.orientation = msg.pose.orientation
#         self.timestamp = self.get_clock().now().to_msg()
#         # frame conversion
#         self.orientation.x *= -1
#         self.orientation.y *= -1
#         self.orientation.z *= -1
#         self.orientation.w *= -1
#         # print statements to know its running
#         print(f"Position: x={self.position.x}, y={self.position.y}, z={self.position.z}")
#         print(f"Orientation: x={self.orientation.x}, y={self.orientation.y}, z={self.orientation.z}, w={self.orientation.w}")
#         print(f"Timestamp: {self.timestamp.sec}.{self.timestamp.nanosec}")
#         print(f"Frame ID: {self.frame_id}")
#         # WRITE BOTH IMMEDIATELY
#         self.send_vision_pose()
#         self.send_setpoint()

#     def send_vision_pose(self):
#         # Create a new PoseStamped message to publish to vision_pose topic
#         vision_pose_msg = PoseStamped()
#         vision_pose_msg.header.stamp = self.timestamp
#         vision_pose_msg.header.frame_id = self.frame_id
#         vision_pose_msg.pose.position = self.position
#         vision_pose_msg.pose.orientation = self.orientation
#         # Publish the message to the /mavros/vision_pose/pose topic
#         self.vision_pose_publisher.publish(vision_pose_msg)

#     def send_setpoint(self):
#         # Create a new PoseStamped message to publish to setpoint topic
#         setpoint_msg = PoseStamped()
#         setpoint_msg.header.stamp = self.timestamp
#         setpoint_msg.header.frame_id = self.frame_id
#         setpoint_msg.pose.position = self.set_position
#         setpoint_msg.pose.orientation = self.set_orientation
#         # Publish the message to the /mavros/setpoint_position/local topic
#         self.setpoint_publisher.publish(setpoint_msg)

#     def callback_waypoints(self, msg):
#         global WAYPOINTS_RECEIVED, WAYPOINTS
#         if WAYPOINTS_RECEIVED:
#             return
#         print('Waypoints Received')
#         WAYPOINTS_RECEIVED = True
#         WAYPOINTS = np.empty((0,3))
#         for pose in msg.poses:
#             pos = np.array([pose.position.x, pose.position.y, pose.position.z])
#             WAYPOINTS = np.vstack((WAYPOINTS, pos))


#     ################################################
#     # HELPER FUNCTIONS
#     ################################################

#     def set_pose_initial(self):
#         # Put the current position into maintained position
#         self.set_position.x = 0.0
#         self.set_position.y = 0.0
#         self.set_position.z = 0.0
#         self.set_orientation.x = 0.0
#         self.set_orientation.y = 0.0
#         self.set_orientation.z = 0.0
#         self.set_orientation.w = -1.0

#     def set_waypoint(self, waypoint):
#         # Put the current position into maintained position
#         self.set_position.x = waypoint[0]
#         self.set_position.y = waypoint[1]
#         self.set_position.z = waypoint[2]
#         self.set_orientation.x = 0.0
#         self.set_orientation.y = 0.0
#         self.set_orientation.z = 0.0
#         self.set_orientation.w = -1.0

#     def close_enough(self, waypoint):
#         # extract the current and waypoint positions
#         curr_x, curr_y, curr_z = self.position.x, self.position.y, self.position.z
#         w_x, w_y, w_z = waypoint[0], waypoint[1], waypoint[2]
#         # check if the drone is close enough to the waypoint
#         # using euclidean distance
#         if np.sqrt((curr_x - w_x)**2 + (curr_y - w_y)**2 + (curr_z - w_z)**2) < self.tol_radius:
#             return True
#         # otherwise, not close enough yet
#         return False


# ################################################
# # MAIN
# ################################################
        
# def main(args=None):

#     # global and local variables
#     global STATE, WAYPOINTS, WAYPOINTS_RECEIVED, set_this_waypoint, waypoint_index
#     test_type = "realsense"

#     # setup the node the run
#     rclpy.init(args=args)
#     node = CommNode(test_type)
#     print("FLIGHT TEST #3 LET'S GO!")
#     rate = node.create_rate(2)
#     # SPIN NODE ONCE AND CHECK FOR WAYPOINTS
#     while rclpy.ok():
#         rclpy.spin_once(node)
#         print("SPIN")
#         if STATE == 'Test':
#             print('Testing...')

#             # check if we're got points and are good to go
#             if WAYPOINTS_RECEIVED:
                
#                 # loop through the waypoints
#                 if waypoint_index < len(WAYPOINTS):

#                     if not set_this_waypoint:
#                         waypoint = WAYPOINTS[waypoint_index]
#                         node.set_waypoint(waypoint)
#                         set_this_waypoint = True

#                     # changing to the next index when close enough
#                     if node.close_enough(waypoint):
#                         set_this_waypoint = False
#                         waypoint_index += 1
#                         print('Waypoint Reached:', waypoint)
#                         print("Next Waypoint:", WAYPOINTS[waypoint_index])
#                 else:
#                     STATE = "Done"
#                     print("Done traversing waypoints!")
#             else:
#                 print('No waypoints received. Cannot test.')

#         # node.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.2))
#         rate.sleep()
#     # JUST SPINNING SHOULD BE FINE
#     # rclpy.spin(comm_node) 
#     # comm_node.destroy_node()

#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()


# ################################################