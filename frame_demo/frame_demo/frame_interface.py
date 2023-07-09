
import math
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Pose


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
