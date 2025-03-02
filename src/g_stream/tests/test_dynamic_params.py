import rclpy
from g_stream.param_dump_manager import param_loader
from rclpy.context import Context

def test_param_loader():
    rclpy.init()
    node = rclpy.create_node("test_node")
    param_loader(node, "/workspace/src/g_stream/config/test.yaml")

    print(node._parameters)
    for name, param in node._parameters.items():
        print(name)
    # for param in node.get_parameters([]):
    #     node.get_logger().warning("dddddddddddddddddddddddddddddddddddd")
    #     print(param.name)

    print("00000000000000000000000000000000000000000000000000000099999999999999999999")
    node.destroy_node()
    rclpy.shutdown()

    assert False


# colcon test --packages-select g_stream --pytest-args -k test_param_load --event-handlers console_direct+
