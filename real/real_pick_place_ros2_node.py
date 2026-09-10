#!/usr/bin/env python3
import importlib.util
import os
import sys
import traceback

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RealPickPlaceNode(Node):
    def __init__(self):
        super().__init__("real_pick_place_node")

        self.status_pub = self.create_publisher(
            String,
            "/real_pick_place/status",
            10,
        )

        default_script = os.path.expanduser(
            "~/colcon_ws/src/robomaster_pick_place_sim/real/pick_place_lua_params.py"
        )
        self.declare_parameter("script_path", default_script)

    def publish_status(self, text):
        msg = String()
        msg.data = str(text)
        self.status_pub.publish(msg)
        self.get_logger().info(str(text))
        rclpy.spin_once(self, timeout_sec=0.05)

    def run_pick_place_script(self):
        script_path = self.get_parameter("script_path").value
        sdk_path = os.path.expanduser("~/RoboMaster-SDK/src")

        if sdk_path not in sys.path:
            sys.path.insert(0, sdk_path)

        if not os.path.exists(script_path):
            raise FileNotFoundError(script_path)

        self.publish_status(f"Loading real script: {script_path}")

        spec = importlib.util.spec_from_file_location(
            "real_pick_place_script",
            script_path,
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        original_step = getattr(module, "step", None)

        if original_step is not None:

            def ros_step(*args, **kwargs):
                if len(args) >= 3:
                    attempt = args[0]
                    step_number = args[1]
                    label = args[2]
                    run_count = getattr(module, "RUN_COUNT", "?")
                    status_text = f"RUN {attempt}/{run_count} | STEP {step_number}: {label}"
                elif len(args) >= 1:
                    status_text = str(args[0])
                else:
                    status_text = "STEP"

                self.publish_status(status_text)
                return original_step(*args, **kwargs)

            module.step = ros_step

        # 把状态发布函数注入脚本, 脚本里的 ERROR/SUCCESS 会同步到话题上
        module.status_hook = self.publish_status

        self.publish_status("Starting real robot pick-place sequence")
        result = module.main()

        if isinstance(result, (tuple, list)) and len(result) == 2:
            success_count, fail_count = result
        else:
            success_count, fail_count = None, None

        if fail_count is None:
            self.publish_status("WARNING: script returned no result (interrupted?)")
        elif fail_count > 0:
            msg = (
                f"ERROR: pick-place failed - "
                f"success={success_count}, failed={fail_count}"
            )
            self.publish_status(msg)
            self.get_logger().error(msg)
        else:
            msg = f"SUCCESS: all {success_count} runs succeeded"
            self.publish_status(msg)
            self.get_logger().info(msg)

        self.publish_status("Real robot pick-place sequence finished")


def main():
    rclpy.init()
    node = RealPickPlaceNode()

    try:
        node.run_pick_place_script()

    except KeyboardInterrupt:
        node.publish_status("Interrupted by user")

    except Exception:
        node.publish_status("Real robot pick-place failed")
        node.get_logger().error(traceback.format_exc())
        raise

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
