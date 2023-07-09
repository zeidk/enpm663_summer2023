
import math
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Pose
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class ListenerDemo(Node):
    '''
    Class to listen to the frames broadcast by the broadcaster.
    '''

    def __init__(self, node_name):
        super().__init__(node_name)

        # Parameter to choose whether to broadcast only or both broadcast and listen
        self._demo_type = self.declare_parameter(
            'demo_type', 'broadcast').get_parameter_value().string_value

        # Parameter to choose the parent frame
        self._parent_frame = self.declare_parameter(
            'parent_frame', 'world').get_parameter_value().string_value

        # Parameter to choose the child frame
        self._child_frame = self.declare_parameter(
            'child_frame', 'first_dynamic_frame').get_parameter_value().string_value

        # Check if the demo_type is valid
        if self._demo_type not in ['broadcast', 'both']:
            raise ValueError('demo_type must be either broadcast or both')

        # Do not execute the listener if the demo_type is broadcast
        if self._demo_type == 'broadcast':
            return

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
        Build the static frames and publish them once.
        '''

        # Generate a random pose
        pose = Pose()
        pose.position.x = 3.5
        pose.position.y = 4.0
        pose.position.z = 5.0

        quaternion = self.quaternion_from_euler(math.pi/2, math.pi/3, math.pi/4)
        pose.orientation.w = quaternion[0]
        pose.orientation.x = quaternion[1]
        pose.orientation.y = quaternion[2]
        pose.orientation.z = quaternion[3]
        self.generate_transform("world", "first_static_frame", pose)

        # Generate a random pose
        pose.position.x = 1.5
        pose.position.y = 2.0
        pose.position.z = 3.0

        quaternion = self.quaternion_from_euler(math.pi/5, math.pi/5, math.pi/5)
        pose.orientation.w = quaternion[0]
        pose.orientation.x = quaternion[1]
        pose.orientation.y = quaternion[2]
        pose.orientation.z = quaternion[3]
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

        quaternion = self.quaternion_from_euler(math.pi/2, math.pi/3, math.pi/4)
        pose.orientation.w = quaternion[0]
        pose.orientation.x = quaternion[1]
        pose.orientation.y = quaternion[2]
        pose.orientation.z = quaternion[3]
        self.generate_transform("world", "first_dynamic_frame", pose)

        # Generate a random pose
        pose.position.x = 1.5
        pose.position.y = 2.0
        pose.position.z = 3.0

        quaternion = self.quaternion_from_euler(math.pi/5, math.pi/5, math.pi/5)
        pose.orientation.w = quaternion[0]
        pose.orientation.x = quaternion[1]
        pose.orientation.y = quaternion[2]
        pose.orientation.z = quaternion[3]
        self.generate_transform("world", "second_dynamic_frame", pose)

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

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion (w in first place)
        quat = [w, x, y, z]
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        quaternion = [0] * 4
        quaternion[0] = cy * cp * cr + sy * sp * sr
        quaternion[1] = cy * cp * sr - sy * sp * cr
        quaternion[2] = sy * cp * sr + cy * sp * cr
        quaternion[3] = sy * cp * cr - cy * sp * sr

        return quaternion
