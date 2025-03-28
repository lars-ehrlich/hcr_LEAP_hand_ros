#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from leap_hand.srv import LeapPosition
from ament_index_python.packages import get_package_share_directory
import numpy as np
import pybullet as p
import pybullet_data
import os
import time
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
from geometry_msgs.msg import PolygonStamped, Point32
from scipy.spatial import ConvexHull
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from launch import LaunchDescription
from launch_ros.actions import Node as LaunchNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

class SimulationNode(Node):
    def __init__(self):
        super().__init__('simulation_node')
        self.positions = np.zeros(16)
        self.init_simulation()
        self.subscription = self.create_subscription(
            JointState,
            '/cmd_ones',
            self.sim_control_callback,
            10
        )
        self.publisher = self.create_publisher(JointState, '/current_state', 10)
        self.orientation_publisher = self.create_publisher(Quaternion, '/current_orientation', 10)
        self.position_publisher = self.create_publisher(Point, '/current_position', 10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/current_pose', 10)
        self.cog_publisher = self.create_publisher(PointStamped, '/current_cog', 10)
        self.cog_plane_publisher = self.create_publisher(PointStamped, '/current_cog_plane', 10)
        self.contact_points_publisher = self.create_publisher(PointStamped, '/contact_points', 10)
        self.polygon_publisher = self.create_publisher(PolygonStamped, '/contact_hull_polygon', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.1, self.publish_hand_state)
        

    def init_simulation(self):
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        plane_id = p.loadURDF('plane.urdf')
        p.changeDynamics(plane_id, -1, lateralFriction=1)

        urdf_file = os.path.join(
            get_package_share_directory('leap_sim'),
            'leap_hand_mesh_right',
            'robot.urdf'
        )
        
        self.LeapId = p.loadURDF(
            urdf_file,
            [0, 0.0, 0.17],
            p.getQuaternionFromEuler([0, 0.4, 3.14]),
            useFixedBase=False
        )

        self.numJoints = p.getNumJoints(self.LeapId)
        p.setGravity(0, 0, -9.8)
        p.setRealTimeSimulation(0)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
        p.setPhysicsEngineParameter(enableConeFriction=1)
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
        p.setRealTimeSimulation(1)

        pose_1 = [ 0.35, -0.46,  0.46,  0.  ,  0.55,  0.  ,  0.45,  0.28,  0.37,  0.28,  0.4 ,  0.3,  1.5, -0, -0.8 ,  0.7 ]
        # set the pose to the initial pose
        for j in range(self.numJoints):
            p.resetJointState(self.LeapId, j, pose_1[j])
        self.positions = pose_1
        

    def sim_control_callback(self, msg):
        self.positions = np.array(msg.position)
        self.update_simulation()

    def update_simulation(self):
        for j in range(self.numJoints):
            p.setJointMotorControl2(self.LeapId, j, p.POSITION_CONTROL, targetPosition=self.positions[j])
        #p.stepSimulation()

    def get_center_of_gravity(self):
        dynamics_info = [p.getDynamicsInfo(self.LeapId, j) for j in range(self.numJoints)]
        masses = [info[0] for info in dynamics_info]
        local_inertial_positions = [info[3] for info in dynamics_info]
        world_positions = [p.getLinkState(self.LeapId, j)[0] for j in range(self.numJoints)]
        
        total_mass = sum(masses)
        cog = np.zeros(3)
        for i in range(self.numJoints):
            cog += np.array(world_positions[i]) * masses[i]
        cog /= total_mass
        return cog

    def get_contact_points(self):
        contact_points = p.getContactPoints(bodyA=self.LeapId)
        points = [contact[5] for contact in contact_points]
        return points

    def publish_hand_state(self):
        msg = JointState()
        msg.position = self.positions.tolist()
        self.publisher.publish(msg)
        
        orientation = p.getBasePositionAndOrientation(self.LeapId)[1]
        orientation_msg = Quaternion()
        orientation_msg.x, orientation_msg.y, orientation_msg.z, orientation_msg.w = orientation
        self.orientation_publisher.publish(orientation_msg)
        
        position = p.getBasePositionAndOrientation(self.LeapId)[0]
        position_msg = Point()
        position_msg.x, position_msg.y, position_msg.z = position
        self.position_publisher.publish(position_msg)
        
        # Publish PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'  # Set the frame_id
        pose_msg.pose.position = position_msg
        pose_msg.pose.orientation = orientation_msg
        self.pose_publisher.publish(pose_msg)

        # Publish center of gravity
        cog = self.get_center_of_gravity()
        cog_msg = PointStamped()
        cog_msg.header.stamp = self.get_clock().now().to_msg()
        cog_msg.header.frame_id = 'map'
        cog_msg.point.x, cog_msg.point.y, cog_msg.point.z = cog

        cog_on_plane_msg = PointStamped()
        cog_on_plane_msg.header.stamp = self.get_clock().now().to_msg()
        cog_on_plane_msg.header.frame_id = 'map'
        cog_on_plane_msg.point.x, cog_on_plane_msg.point.y, cog_on_plane_msg.point.z = cog
        cog_on_plane_msg.point.z = 0.0

        self.cog_publisher.publish(cog_msg)
        self.cog_plane_publisher.publish(cog_on_plane_msg)


        # Publish contact points
        contact_points = self.get_contact_points()

        # Publish contact polygon (convex hull)
        if len(contact_points) > 3:
            hull = ConvexHull(contact_points)
            polygon_msg = PolygonStamped()
            polygon_msg.header.stamp = self.get_clock().now().to_msg()
            polygon_msg.header.frame_id = 'map'
            for vertex in hull.vertices:
                point = Point32()
                point.x, point.y, point.z = contact_points[vertex]
                polygon_msg.polygon.points.append(point)
            self.polygon_publisher.publish(polygon_msg)
        elif len(contact_points) == 3:
            polygon_msg = PolygonStamped()
            polygon_msg.header.stamp = self.get_clock().now().to_msg()
            polygon_msg.header.frame_id = 'map'
            for point in contact_points:
                print(point)
                out_point = Point32()
                out_point.x, out_point.y, out_point.z = point
                polygon_msg.polygon.points.append(out_point)
            self.polygon_publisher.publish(polygon_msg)
        else:
            self.get_logger().info('Not enough contact points to compute convex hull')


        # Publish TF2 transforms for all joints
        for j in range(self.numJoints):
            link_state = p.getLinkState(self.LeapId, j)
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = 'map'
            transform.child_frame_id = f'joint_{j}'
            transform.transform.translation.x = link_state[0][0]
            transform.transform.translation.y = link_state[0][1]
            transform.transform.translation.z = link_state[0][2]
            transform.transform.rotation.x = link_state[1][0]
            transform.transform.rotation.y = link_state[1][1]
            transform.transform.rotation.z = link_state[1][2]
            transform.transform.rotation.w = link_state[1][3]
            self.tf_broadcaster.sendTransform(transform)
            

def main(args=None):    
    rclpy.init(args=args)
    simulation_node = SimulationNode()
    rclpy.spin(simulation_node)
    simulation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()