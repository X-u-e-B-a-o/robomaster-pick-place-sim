#!/usr/bin/env python3
"""RoboMaster EP 真机驱动节点.

对外伪装成仿真阶段的 ros2_control 控制器栈, 让任务节点 (pick_place_moveit)
与 move_group 原样复用, 满足实验要求 "真机阶段复用仿真阶段的 ROS 2 接口,
仅更换设备与参数配置":

  - /joint_states                            sensor_msgs/JointState 发布 (仿真 5 关节名)
  - /arm_controller/follow_joint_trajectory  control_msgs FollowJointTrajectory Action 服务
  - /gripper_controller/commands             Float64MultiArray 订阅 (开度位置指令)
  - /controller_manager/list_controllers     伪服务 (三个控制器恒为 active)
  - /ep_arm/freeze                           std_srvs/SetBool 急停/解冻 (停止下发指令)

运动映射:
  真机臂是 2 自由度平面臂 (官方 SDK 只有 moveto(x, y) mm, 无偏航轴,
  工作范围 x∈[0,220]mm 前向、y∈[0,150]mm 高度), 因此仿真轨迹中的
  base_yaw_joint 被忽略, 两个俯仰关节按下式映射到真机坐标:

    waypoint(lift, wrist) --仿真平面FK--> (x_s, z_s)
        x_s = SHOULDER_X + L1*cos(lift) + L2*cos(lift+wrist)
        z_s = SHOULDER_Z - L1*sin(lift) - L2*sin(lift+wrist)
    --标定仿射映射--> x_r = kx*x_s + bx,  y_r = kz*z_s + bz   [mm]

  仿射系数由两个标定锚点决定 (A 取物点、B 放置点在仿真空间与真机空间
  的一一对应, 用 calibrate_real 工具标定后写入参数文件)。

执行约束:
  官方 SDK 同一时刻只允许机械臂执行一个动作 (重复下发会抛异常),
  因此收到轨迹后先抽稀 (相邻 waypoint 真机距离 >= waypoint_min_dist_mm),
  再逐个 moveto + wait_for_completed 顺序执行。

安全:
  - 映射结果越出真机工作范围 -> 拒绝执行 (INVALID_GOAL) 并记录错误
  - /ep_arm/freeze -> 停止下发新指令, 当前动作完成后保持位置
    (真机硬件急停仍以断电装置为准, 见 docs/real_machine_runbook.md)
  - dry_run:=true -> 不连接真机、不下发任何指令, 只打印抽稀后的
    真机 (x,y) 路径, 供正式运行前安全检查
"""
import math
import os
import threading
import time

import yaml

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import SetBool

# 仿真运动学常量与 robomaster_pick_place_sim 包保持一致 (不能本地复制,
# 否则 URDF 改动后常量漂移, 任务节点 run() 的 FK 校验会拦住)
from robomaster_pick_place_sim.robot_kinematics import (ARM_L1, ARM_L2,
                                                        ARM_SHOULDER)

ARM_JOINTS = ['base_yaw_joint', 'arm_lift_joint', 'wrist_pitch_joint']
FINGER_JOINTS = ['left_finger_joint', 'right_finger_joint']
CONTROLLER_NAMES = ['joint_state_broadcaster', 'arm_controller',
                    'gripper_controller']

DEFAULT_PARAMS = {
    'conn_type': 'ap',           # ap=热点直连 sta=组网 rndis=USB
    'robot_sn': '',              # sta 组网模式才需要 SN
    'dry_run': False,            # true: 不控制真机, 只打印映射路径
    'joint_state_rate': 20.0,    # /joint_states 发布频率 [Hz]
    'waypoint_min_dist_mm': 4.0, # 轨迹抽稀阈值: 真机距离 [mm]
    'moveto_timeout_s': 5.0,     # 单个 waypoint 等待完成超时 [s]
    'gripper_close_threshold': 0.02,  # 开度指令 <= 该值 -> 夹紧 [m]
    'gripper_open_power': 60,    # 张开出力 [1,100]
    'gripper_close_power': 40,   # 夹紧出力 [1,100] (目标物 <=100g)
    # 标定锚点: A/B 点在仿真空间 (x_s, z_s) 与真机空间 (x_r, y_r) [mm] 的对应,
    # 仿射映射系数在启动时由此计算; anchor*_real 由 calibrate_real 标定写入
    'anchor1_sim': [0.60, 0.32], 'anchor1_real': [60, 40],
    'anchor2_sim': [0.64, 0.35], 'anchor2_real': [170, 60],
    'home_real': [32, 114],      # 真机回零点 [mm] (仅日志/校验用)
    'arm_x_range': [0.0, 220.0], # 官方规格: 水平 0~0.22 m
    'arm_y_range': [0.0, 150.0], # 官方规格: 垂直 0~0.15 m
}


class EPArmDriver(Node):
    def __init__(self):
        super().__init__('ep_arm_driver')
        self.declare_parameter('params_file', '')
        self.declare_parameter('dry_run', False)
        self.p = self._load_params()

        # ---------- 运行状态 ----------
        self._robot = None
        self._arm = None
        self._gripper = None
        self._connected = False
        self._frozen = False
        self._connect_lock = threading.Lock()
        self._arm_pos = None              # 最新真机 (x, y) [mm] (push 数据)
        self._last_lift = 0.0             # 最后下发的 lift (供 joint_states)
        self._last_wrist = 0.0
        self._last_finger = 0.040         # 最后下发的夹爪开度 [m]
        self._grip_state = 'open'         # 'open' | 'close'

        # ---------- 映射系数 (标定锚点 -> 仿射) ----------
        (self._kx, self._bx), (self._kz, self._bz) = self._compute_map()

        # ---------- 对外接口 ----------
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.status_pub = self.create_publisher(String, '/ep_arm/status', 10)
        self.create_subscription(Float64MultiArray,
                                 '/gripper_controller/commands',
                                 self._grip_cb, 10)
        self.create_service(SetBool, '/ep_arm/freeze', self._freeze_cb)
        self.create_service(ListControllers,
                            '/controller_manager/list_controllers',
                            self._list_controllers_cb)

        # Action 服务: 一次只执行一个轨迹目标.
        # Humble 的 ActionServer 把 execute_callback 调度成独立的 executor
        # task, 不占本回调组, 因此执行期间 cancel/goal 回调仍能被处理
        # (SDK 无法中断在途动作, 但当前 waypoint 完成后立即停止).
        self._action_cbg = MutuallyExclusiveCallbackGroup()
        self._action_server = ActionServer(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            self._execute_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._action_cbg)

        # ---------- joint_states 定时发布 ----------
        self.create_timer(1.0 / self.p['joint_state_rate'], self._pub_js)

        self.status('EP 驱动启动 (dry_run={})'.format(self.p['dry_run']))

        # ---------- SDK 连接 (后台线程, 失败自动重试) ----------
        if not self.p['dry_run']:
            t = threading.Thread(target=self._connect_thread, daemon=True)
            t.start()

    # ================= 参数 =================

    def _load_params(self):
        params = dict(DEFAULT_PARAMS)
        path = self.get_parameter('params_file').value
        if not path:
            path = os.path.join(
                get_package_share_directory('robomaster_ep_driver'),
                'config', 'ep_driver_params.yaml')
        if os.path.exists(path):
            with open(path) as f:
                data = yaml.safe_load(f)
            params.update(data.get('ep_arm_driver', {}).get(
                'ros__parameters', {}))
            self.get_logger().info(f'参数文件: {path}')
        else:
            self.get_logger().warn(f'参数文件不存在, 使用默认值: {path}')
        # launch 参数优先于参数文件 (dry_run:=true 必须能覆盖文件里的 false)
        params['dry_run'] = self.get_parameter('dry_run').value
        return params

    def _compute_map(self):
        """由标定锚点计算仿真->真机仿射映射系数."""
        a1s, a1r = self.p['anchor1_sim'], self.p['anchor1_real']
        a2s, a2r = self.p['anchor2_sim'], self.p['anchor2_real']
        for axis in ('x', 'z'):
            if abs(a2s[0 if axis == 'x' else 1] -
                   a1s[0 if axis == 'x' else 1]) < 1e-6:
                self.get_logger().error(f'标定锚点 {axis} 方向仿真坐标重合, '
                                        f'无法计算映射')
        kx = (a2r[0] - a1r[0]) / (a2s[0] - a1s[0])
        bx = a1r[0] - kx * a1s[0]
        kz = (a2r[1] - a1r[1]) / (a2s[1] - a1s[1])
        bz = a1r[1] - kz * a1s[1]
        self.get_logger().info(
            f'标定映射: x_r={kx:.1f}*x_s{bx:+.1f}, y_r={kz:.1f}*z_s{bz:+.1f} [mm]')
        return (kx, bx), (kz, bz)

    def status(self, msg):
        self.get_logger().info(msg)
        self.status_pub.publish(String(data=msg))

    # ================= SDK 连接 =================

    def _connect_thread(self):
        while rclpy.ok() and not self._connected:
            try:
                # 延迟导入: dry_run 或未装 SDK 的机器上不影响其它功能
                from robomaster import robot as rm_robot
                self.status(f'正在连接 EP (conn_type={self.p["conn_type"]}) ...')
                robot = rm_robot.Robot()
                robot.initialize(conn_type=self.p['conn_type'],
                                 sn=self.p['robot_sn'] or None)
                arm = robot.robotic_arm
                arm.sub_position(freq=10, callback=self._arm_pos_cb)
                gripper = robot.gripper
                with self._connect_lock:
                    self._robot = robot
                    self._arm = arm
                    self._gripper = gripper
                    self._connected = True
                self.status('EP 连接成功')
            except Exception as e:  # noqa: BLE001
                self.status(f'EP 连接失败 (10s 后重试): {e}')
                time.sleep(10)

    def _arm_pos_cb(self, pos):
        self._arm_pos = pos

    def _ensure_connected(self):
        with self._connect_lock:
            return self._connected

    # ================= 夹爪 =================

    def _grip_cb(self, msg):
        if len(msg.data) < 2:
            return
        v = msg.data[0]
        want = 'close' if v <= self.p['gripper_close_threshold'] else 'open'
        if want == self._grip_state:
            self._last_finger = v
            return
        if self.p['dry_run']:
            self.status(f'[dry_run] 夹爪指令 {v:.3f} m -> {want}')
        elif self._ensure_connected():
            power = (self.p['gripper_close_power'] if want == 'close'
                     else self.p['gripper_open_power'])
            ok = self._gripper.close(power=power) if want == 'close' \
                else self._gripper.open(power=power)
            if not ok:
                self.get_logger().error(f'夹爪 {want} 指令下发失败')
        else:
            self.get_logger().error('EP 未连接, 夹爪指令被忽略')
        self._grip_state = want
        self._last_finger = v

    # ================= 服务 =================

    def _freeze_cb(self, request, response):
        self._frozen = bool(request.data)
        if self._frozen:
            self.status('急停冻结: 停止下发新指令, 当前动作完成后保持位置')
        else:
            self.status('解除冻结')
        response.success = True
        response.message = ('frozen' if self._frozen else 'unfrozen')
        return response

    def _list_controllers_cb(self, request, response):
        # 伪服务: 让任务节点 _wait_for_system 的控制器激活检查通过
        for name in CONTROLLER_NAMES:
            cs = ControllerState()
            cs.name = name
            cs.state = 'active'
            response.controller.append(cs)
        return response

    # ================= joint_states =================

    def _pub_js(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = (ARM_JOINTS + FINGER_JOINTS)
        # 真机无偏航轴, base_yaw 恒为 0; 俯仰关节发布最后下发的指令值
        msg.position = [0.0, self._last_lift, self._last_wrist,
                        self._last_finger, self._last_finger]
        self.js_pub.publish(msg)

    # ================= 轨迹 Action =================

    def _cancel_cb(self, goal_handle):
        self.status('轨迹目标被取消 (move_group 急停), 停止下发新指令')
        return CancelResponse.ACCEPT

    def _to_real(self, lift, wrist):
        """仿真平面 FK -> 标定映射 -> 真机 (x, y) [mm]."""
        x_s = (ARM_SHOULDER[0] + ARM_L1 * math.cos(lift)
               + ARM_L2 * math.cos(lift + wrist))
        z_s = (ARM_SHOULDER[1] - ARM_L1 * math.sin(lift)
               - ARM_L2 * math.sin(lift + wrist))
        x_r = self._kx * x_s + self._bx
        y_r = self._kz * z_s + self._bz
        return x_r, y_r

    def _execute_cb(self, goal_handle):
        goal = goal_handle.request
        result = FollowJointTrajectory.Result()

        traj = goal.trajectory
        if not traj.points or not traj.joint_names:
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = '空轨迹'
            goal_handle.abort()
            return result

        # 按名称取关节值 (yaw 忽略, 真机无偏航轴)
        try:
            idx = {n: i for i, n in enumerate(traj.joint_names)}
            lift_i = idx['arm_lift_joint']
            wrist_i = idx['wrist_pitch_joint']
        except KeyError as e:
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            result.error_string = f'轨迹缺少关节 {e}'
            goal_handle.abort()
            return result

        # 映射 + 范围校验 + 抽稀
        last = None
        waypoints = []   # [(x_r, y_r, lift, wrist)]
        x_lo, x_hi = self.p['arm_x_range']
        y_lo, y_hi = self.p['arm_y_range']
        n_pts = len(traj.points)
        for i, wp in enumerate(traj.points):
            lift = wp.positions[lift_i]
            wrist = wp.positions[wrist_i]
            x_r, y_r = self._to_real(lift, wrist)
            if not (x_lo <= x_r <= x_hi and y_lo <= y_r <= y_hi):
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = (
                    f'waypoint {i}: 真机坐标 ({x_r:.0f},{y_r:.0f})mm '
                    f'超出工作范围 [{x_lo},{x_hi}]x[{y_lo},{y_hi}]')
                self.get_logger().error(result.error_string)
                goal_handle.abort()
                return result
            final = (i == n_pts - 1)
            if (last is None or final or
                    math.hypot(x_r - last[0], y_r - last[1])
                    >= self.p['waypoint_min_dist_mm']):
                waypoints.append((x_r, y_r, lift, wrist))
                last = (x_r, y_r)

        self.status(f'轨迹目标: {n_pts} 个 waypoint -> '
                    f'抽稀后 {len(waypoints)} 个真机目标点')
        if self.p['dry_run']:
            self.status('[dry_run] 真机路径预览:')
            for x_r, y_r, _, _ in waypoints:
                self.status(f'[dry_run]   moveto({x_r:.0f}, {y_r:.0f}) mm')

        # 顺序执行 (SDK 同一时刻只允许一个机械臂动作)
        for i, (x_r, y_r, lift, wrist) in enumerate(waypoints):
            if goal_handle.is_cancel_requested or self._frozen:
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = '轨迹被取消/急停冻结'
                goal_handle.canceled()
                self.status('轨迹执行中止 (取消/冻结)')
                return result

            if self.p['dry_run']:
                time.sleep(0.02)
            else:
                if not self._ensure_connected():
                    result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                    result.error_string = 'EP 未连接'
                    goal_handle.abort()
                    return result
                try:
                    act = self._arm.moveto(x=int(round(x_r)),
                                           y=int(round(y_r)))
                except Exception as e:  # noqa: BLE001
                    result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                    result.error_string = f'moveto({x_r:.0f},{y_r:.0f}) 下发失败: {e}'
                    self.get_logger().error(result.error_string)
                    goal_handle.abort()
                    return result
                if not act.wait_for_completed(
                        timeout=self.p['moveto_timeout_s']):
                    result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                    result.error_string = (
                        f'moveto({x_r:.0f},{y_r:.0f}) 超时未完成')
                    self.get_logger().error(result.error_string)
                    goal_handle.abort()
                    return result

            self._last_lift, self._last_wrist = lift, wrist
            fb = FollowJointTrajectory.Feedback()
            fb.desired.positions = [0.0, lift, wrist]
            goal_handle.publish_feedback(fb)
            if i % 5 == 0 or i == len(waypoints) - 1:
                self.status(f'执行 {i+1}/{len(waypoints)}: '
                            f'moveto({x_r:.0f},{y_r:.0f}) mm 完成')

        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = ''
        goal_handle.succeed()
        self.status('轨迹执行完成')
        return result


def main():
    rclpy.init()
    node = EPArmDriver()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # SIGTERM/SIGINT 时 rclpy 自身的信号处理可能已关闭 context,
        # 重复 shutdown 会抛 RCLError, 退出阶段无需在意
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
