import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from rcl_interfaces.msg import SetParametersResult

class InstructionNode(Node):
    def __init__(self):
        super().__init__('instruction_node')
        
        # Configure a Latched/Transient Local publisher with explicit Reliability
        qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.publisher = self.create_publisher(String, '/vla/language_instruction', qos_profile)
        
        # Declare the parameter 'instruction' with an empty default value
        self.declare_parameter('instruction', '')
        
        # Trigger publish if a default/initial parameter is passed at startup
        initial_instruction = self.get_parameter('instruction').get_parameter_value().string_value
        if initial_instruction:
            self.publish_instruction(initial_instruction)
            
        # Add callback to listen for parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        self.get_logger().info("Instruction node is ready!")
        self.get_logger().info("Set the instruction using: ros2 param set /instruction_node instruction 'your task description'")

    def publish_instruction(self, text):
        self.get_logger().info(f"Publishing language instruction: '{text}'")
        msg = String()
        msg.data = text
        self.publisher.publish(msg)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'instruction' and param.type_ == rclpy.Parameter.Type.STRING:
                self.publish_instruction(param.value)
                
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = InstructionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

