
import math
from typing import Tuple
import rclpy
import PyKDL
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Pose, Quaternion
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# For type hints


def rpy_from_quaternion(quaternion: Quaternion) -> Tuple[float, float, float]:
    ''' 
    Use KDL to convert a quaternion to euler angles roll, pitch, yaw.
    Args:
        quaternion (Quaternion): quaternion to convert
    Returns:
        Tuple[float, float, float]: A tuple containing roll, pitch, and yaw
    '''

    rotation = PyKDL.Rotation.Quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    return rotation.GetRPY()


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    '''
    Converts euler roll, pitch, yaw to a tuple of quaternion values (JPL convention).

    Returns:
        Quaternion: tuple of quaternion values (JPL convention)
    '''
    quaternion = PyKDL.Rotation.RPY(roll, pitch, yaw).GetQuaternion()

    # Other way to do it
    # cy = math.cos(yaw * 0.5)
    # sy = math.sin(yaw * 0.5)
    # cp = math.cos(pitch * 0.5)
    # sp = math.sin(pitch * 0.5)
    # cr = math.cos(roll * 0.5)
    # sr = math.sin(roll * 0.5)

    # quaternion = [0] * 4
    # quaternion[0] = cy * cp * cr + sy * sp * sr
    # quaternion[1] = cy * cp * sr - sy * sp * cr
    # quaternion[2] = sy * cp * sr + cy * sp * cr
    # quaternion[3] = sy * cp * cr - cy * sp * sr

    return quaternion


class KDLFrameDemo(Node):
    '''
    Class showing how to use KDL Frame to compute the pose of a part in the 'world' frame.
    '''

    def __init__(self, node_name):
        super().__init__(node_name)

        # Get the kdl parameter
        self._kdl = self.declare_parameter(
            'kdl', False).get_parameter_value().bool_value
        """Parameter kdl to enable/disable the KDL demo. Default is false."""
        
        # Do not execute the demo if kdl is false
        if not self._kdl:
            self.get_logger().warn("KDL demo is not started.")
            return

        self.get_logger().info('KDL demo started')

        self._camera_pose_in_world = self.set_camera_pose_in_world()
        self._part_pose_in_camera = self.set_part_pose_in_camera()
        self._part_pose_in_world = self.multiply_pose(self._camera_pose_in_world, self._part_pose_in_camera)

        self._rpy = rpy_from_quaternion(self._part_pose_in_world.orientation)

        self.get_logger().info(
            f'Part position in world frame: \n x: {self._part_pose_in_world.position.x}, y: {self._part_pose_in_world.position.y}, z: {self._part_pose_in_world.position.z}')
        self.get_logger().info(
            f'Part orientation in world frame: \n rpy: {self._rpy[0]}, {self._rpy[1]}, {self._rpy[2]}')

    def set_camera_pose_in_world(self):
        '''
        Set the pose of the camera in the world frame.
        Information about the camera pose can be obtained in two different ways:
        
            - From the sensors.yaml file
            - By clicking on the camera in gazebo.

        Returns:
            Pose: The pose of the camera in the world frame
        '''
        
        # Create a pose object for right_bins_camera
        pose = Pose()
        pose.position.x = -2.286
        pose.position.y = 2.96
        pose.position.z = 1.8

        quat_x, quat_y, quat_z, quat_w = quaternion_from_euler(math.pi, math.pi/2, 0)
        pose.orientation.x = quat_x
        pose.orientation.y = quat_y
        pose.orientation.z = quat_z
        pose.orientation.w = quat_w

        return pose

    def set_part_pose_in_camera(self):
        '''
        Set the pose of the part detected by the camera.
        Information about the part pose should be obtained from the camera subscriber.
        Here we are hardcoding this information for the sake of simplicity.
        The hardcoded pose was retrieved directly from gazebo.

        Returns:
            Pose: The pose of the part in the camera frame
        '''
        pose = Pose()
        pose.position.x = 1.0769784427063858
        pose.position.y = 0.15500024548461805
        pose.position.z = -0.5660066794253416

        pose.orientation.x = -0.0013258319361092675
        pose.orientation.y = -0.7083612841685344
        pose.orientation.z = -0.0013332542580505244
        pose.orientation.w = 0.7058475442288268

        return pose

    def multiply_pose(self, pose1: Pose, pose2: Pose) -> Pose:
        '''
        Use KDL to multiply two poses together.
        Args:
            p1 (Pose): Pose of the first frame
            p2 (Pose): Pose of the second frame
        Returns:
            Pose: Pose of the resulting frame
        '''

        orientation1 = pose1.orientation
        frame1 = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(orientation1.x, orientation1.y, orientation1.z, orientation1.w),
            PyKDL.Vector(pose1.position.x, pose1.position.y, pose1.position.z))

        orientation2 = pose2.orientation
        frame2 = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(orientation2.x, orientation2.y, orientation2.z, orientation2.w),
            PyKDL.Vector(pose2.position.x, pose2.position.y, pose2.position.z))

        frame3 = frame1 * frame2

        # return the resulting pose from frame3
        pose = Pose()
        pose.position.x = frame3.p.x()
        pose.position.y = frame3.p.y()
        pose.position.z = frame3.p.z()

        q = frame3.M.GetQuaternion()
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        return pose


class ListenerDemo(Node):
    '''
    Class to listen to the frames broadcast by the broadcaster.
    '''

    def __init__(self, node_name):
        super().__init__(node_name)

        # Get the listen parameter
        self._listen = self.declare_parameter(
            'listen', False).get_parameter_value().bool_value

        # Parameter to choose the parent frame
        self._parent_frame = self.declare_parameter(
            'parent_frame', 'world').get_parameter_value().string_value

        # Parameter to choose the child frame
        self._child_frame = self.declare_parameter(
            'child_frame', 'first_dynamic_frame').get_parameter_value().string_value

        # Do not execute the demo if listen is false
        if not self._listen:
            self.get_logger().warn("Listener demo is not started.")
            return

        self.get_logger().info('Listener demo started')

        # Create a transform buffer and listener
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Listen to the transform between frames periodically
        self._listener_timer = self.create_timer(1, self._listener_cb)

    def _listener_cb(self):
        '''
        Callback function for the listener timer.
        '''
        try:
            # Get the transform between frames
            transform = self._tf_buffer.lookup_transform(
                self._parent_frame, self._child_frame, rclpy.time.Time())
            self.get_logger().info(
                f"Transform between {self._parent_frame} and {self._child_frame}: \n" + str(transform))
        except TransformException as ex:
            self.get_logger().fatal(
                f"Could not get transform between {self._parent_frame} and {self._child_frame}: {str(ex)}")


class BroadcasterDemo(Node):
    '''
    Class to broadcast Frames. This class consists of a static broadcaster and a dynamic broadcaster.
    '''

    def __init__(self, node_name):
        super().__init__(node_name)

        # Get the broadcast parameter
        self._broadcast = self.declare_parameter(
            'broadcast', True).get_parameter_value().bool_value
        # Do not execute the demo if broadcast is false
        if not self._broadcast:
            self.get_logger().warn("Broadcaster demo is not started.")
            return

        self.get_logger().info('Broadcaster demo started')

        self.transforms = []

        # Create a static broadcaster
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        # Publish the static frames once
        self.publish_static_frames()

        # Create a dynamic broadcaster
        self.tf_dynamic_broadcaster = TransformBroadcaster(self)
        # Publish the dynamic frames periodically because they are changing with time
        self._dynamic_broadcaster_timer = self.create_timer(
            0.005, self.dynamic_broadcaster_cb)

    def publish_static_frames(self):
        '''
        Build two static frames and publish them once.
        '''

        # Generate a first arbitrary pose
        pose = Pose()
        pose.position.x = 3.5
        pose.position.y = 4.0
        pose.position.z = 5.0

        quat_x, quat_y, quat_z, quat_w = quaternion_from_euler(math.pi/2, math.pi/3, math.pi/4)
        pose.orientation.x = quat_x
        pose.orientation.y = quat_y
        pose.orientation.z = quat_z
        pose.orientation.w = quat_w

        self.generate_transform("world", "first_static_frame", pose)

        # Generate a second arbitrary pose
        pose.position.x = 1.5
        pose.position.y = 2.0
        pose.position.z = 3.0

        quat_x, quat_y, quat_z, quat_w = quaternion_from_euler(math.pi/5, math.pi/5, math.pi/5)
        pose.orientation.x = quat_x
        pose.orientation.y = quat_y
        pose.orientation.z = quat_z
        pose.orientation.w = quat_w

        self.generate_transform("world", "second_static_frame", pose)
        self.tf_static_broadcaster.sendTransform(self.transforms)

    def dynamic_broadcaster_cb(self):
        '''
        Callback function for the dynamic broadcaster timer.
        '''

        self.transforms.clear()

        # Generate a random pose
        pose = Pose()
        pose.position.x = 3.5
        pose.position.y = 4.0
        pose.position.z = 5.0

        quat_x, quat_y, quat_z, quat_w = quaternion_from_euler(math.pi/2, math.pi/3, math.pi/4)
        pose.orientation.x = quat_x
        pose.orientation.y = quat_y
        pose.orientation.z = quat_z
        pose.orientation.w = quat_w
        self.generate_transform("world", "dynamic_frame", pose)

        self.send_dynamic_transforms()

    def generate_transform(self, parent, child, pose):
        '''
        Build a transform message and append it to the list of transforms to be broadcast.

        Args:
            parent (str): Parent frame of the child frame.
            child (str): Child frame of the parent frame.
            pose (geometry_msgs.msg.Pose): Pose of the child frame with respect to the parent frame.

        '''
        transform_stamped = TransformStamped()

        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = parent
        transform_stamped.child_frame_id = child

        transform_stamped.transform.translation.x = pose.position.x
        transform_stamped.transform.translation.y = pose.position.y
        transform_stamped.transform.translation.z = pose.position.z
        transform_stamped.transform.rotation.x = pose.orientation.x
        transform_stamped.transform.rotation.y = pose.orientation.y
        transform_stamped.transform.rotation.z = pose.orientation.z
        transform_stamped.transform.rotation.w = pose.orientation.w

        self.transforms.append(transform_stamped)

    def send_dynamic_transforms(self):
        '''
        Publish the transforms in the list of transforms to be broadcast.
        '''
        self.tf_dynamic_broadcaster.sendTransform(self.transforms)
