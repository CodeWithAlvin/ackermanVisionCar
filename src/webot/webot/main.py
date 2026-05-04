import rclpy
from rclpy.executors import MultiThreadedExecutor

from webot.perception import PerceptionNode
from webot.control import ControlNode

def main(args=None):
    rclpy.init(args=args)
    
    perception = PerceptionNode()
    controller = ControlNode()

    executor = MultiThreadedExecutor()
    executor.add_node(perception)
    executor.add_node(controller)

    print("Webot Master Process Started. Spinning Perception and Control nodes...")

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("Shutting down nodes...")
    finally:
        executor.shutdown()
        perception.destroy_node()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()