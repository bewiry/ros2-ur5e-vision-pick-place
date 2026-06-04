import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

class BoxSubscriber(Node):
    def __init__(self):
        super().__init__('box_subscriber')
        
        # Subscribe to the exact topic your tracker is publishing to
        self.subscription = self.create_subscription(
            PointStamped,
            '/conveyor/object_position',
            self.listener_callback,
            10)
            
        self.get_logger().info("Robot Kinematics Node Started. Waiting for target coordinates...")

    def listener_callback(self, msg):
        # Extract the data from the message
        color = msg.header.frame_id
        x = msg.point.x
        y = msg.point.y
        z = msg.point.z
        
        # Print a clean, formatted log message to the terminal
        self.get_logger().info(f"Target Acquired -> Color: {color} | X: {x:.2f} cm | Y: {y:.2f} cm | Z: {z:.2f} cm")

def main(args=None):
    rclpy.init(args=args)
    node = BoxSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
