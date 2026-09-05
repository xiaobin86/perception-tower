import os
import tempfile
import time
import pytest
import rclpy
from rclpy.node import Node
from perception_tower_interfaces.srv import TowerCommand
from perception_tower_interfaces.msg import TowerStatus
from sensor_msgs.msg import PointCloud2, Image


@pytest.fixture
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def wait_for_status(node, state_value, timeout_s=10.0):
    received = []
    sub = node.create_subscription(TowerStatus, '/perception_tower/status', lambda m: received.append(m.state), 10)
    start = time.time()
    while time.time() - start < timeout_s:
        rclpy.spin_once(node, timeout_sec=0.1)
        if received and received[-1] == state_value:
            sub.destroy()
            return True
    sub.destroy()
    return False


def test_mock_scan_end_to_end(ros_context):
    from perception_tower.tower_node import TowerNode
    out = tempfile.mkdtemp()
    node = TowerNode(
        parameter_overrides=[
            ('mock_hardware', True),
            ('output_dir', out),
            ('serial_port', ''),
        ]
    )
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    import threading
    t = threading.Thread(target=executor.spin, daemon=True)
    t.start()

    client_node = rclpy.create_node('test_client')
    cli = client_node.create_client(TowerCommand, '/perception_tower/command')
    assert cli.wait_for_service(timeout_sec=3.0)

    def call(cmd):
        req = TowerCommand.Request()
        req.command = cmd
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(client_node, fut, timeout_sec=2.0)
        return fut.result()

    resp = call(TowerCommand.Request.CMD_INIT)
    assert resp.accepted
    assert wait_for_status(node, TowerStatus.READY, timeout_s=10.0)

    cloud_received = []
    node.create_subscription(PointCloud2, '/perception_tower/stitched_points', lambda m: cloud_received.append(m), 10)
    img_received = []
    node.create_subscription(Image, '/perception_tower/photo_color', lambda m: img_received.append(m), 10)

    resp = call(TowerCommand.Request.CMD_SCAN)
    assert resp.accepted
    assert wait_for_status(node, TowerStatus.READY, timeout_s=30.0)

    assert len(cloud_received) >= 1
    assert len(img_received) >= 1
    assert os.path.exists(os.path.join(out, os.listdir(out)[0], 'color.png'))

    executor.shutdown()
    node.destroy_node()
    client_node.destroy_node()
