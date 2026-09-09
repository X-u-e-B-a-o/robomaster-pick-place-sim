#!/usr/bin/env python3
"""RoboMaster EP 定点抓取节点 (MoveIt 2 版)

流程: 回零 -> 取物点A上方(预抓取) -> 沿手指方向接近 -> 夹取 ->
      抬升 -> 放置点B上方(预放置) -> 沿手指方向接近 -> 释放 -> 回零

- 手臂运动由 MoveIt move_group 规划并经由 arm_controller 的
  FollowJointTrajectory Action 执行
- 夹爪通过 /gripper_controller/commands 话题直接控制
- 每次规划/执行失败(不可达、无逆解、关节超限)都记录错误、急停、
  返回安全位置; 日志/轨迹/结果保存到 log_dir
"""
import csv
import json
import math
import os
import sys
import time
from datetime import datetime

import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from controller_manager_msgs.srv import ListControllers
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Float64MultiArray, String

import moveit_commander
from moveit_commander import MoveGroupCommander

# 与 robomaster_ep_moveit_config/config/joint_limits.yaml 保持一致
JOINT_LIMITS = {
    'base_yaw_joint': (-3.14, 3.14),
    'arm_lift_joint': (-0.8, 1.0),
    'wrist_pitch_joint': (-1.5, 1.5),
    'left_finger_joint': (0.0, 0.045),
    'right_finger_joint': (0.0, 0.045),
}

DEFAULT_PARAMS = {
    'num_cycles': 5,
    'gripper_open': 0.040,
    'gripper_closed': 0.006,
    'approach_dist': 0.12,
    'lift_dist': 0.12,
    'max_velocity_scale': 0.5,
    'max_acceleration_scale': 0.5,
    'pick_point': {'x': 0.60, 'y': 0.0, 'z': 0.337},
    'place_point': {'x': 0.35, 'y': 0.30, 'z': 0.337},
    'attach_object': True,
    'finger_center_offset': 0.06,
    'object_size': 0.07,
    'table': {'center': [0.75, 0.0, 0.15], 'size': [0.90, 0.70, 0.30]},
    'log_dir': 'results',
    'abort_on_error': False,
}


class TaskError(Exception):
    """一次抓取循环中的可恢复错误."""


class PickPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_place_node')
        self.declare_parameter('params_file', '')
        self.declare_parameter('use_sim_time', True)

        self.p = self._load_params()

        # ---------- 日志与结果文件 ----------
        self.log_dir = os.path.abspath(self.p['log_dir'])
        os.makedirs(self.log_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_path = os.path.join(self.log_dir, f'run_{stamp}.log')
        self.csv_path = os.path.join(self.log_dir, f'run_{stamp}_trajectory.csv')
        self.json_path = os.path.join(self.log_dir, f'run_{stamp}_results.json')
        self.cycle = 0
        self.successes = 0
        self.failures = 0
        self.cycle_records = []
        self.csv_f = open(self.csv_path, 'w', newline='')
        self.csv_w = csv.writer(self.csv_f)
        self.csv_w.writerow(['timestamp', 'cycle', 'step', 'base_yaw', 'arm_lift',
                             'wrist_pitch', 'left_finger', 'right_finger',
                             'ee_x', 'ee_y', 'ee_z', 'success'])

        # ---------- 发布器 / 订阅 ----------
        self.grip_pub = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.status_pub = self.create_publisher(String, '/pick_place/status', 10)
        self._last_js = None
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

        # ---------- MoveIt ----------
        if not rclpy.ok():
            moveit_commander.roscpp_initialize(sys.argv)
        self.group = MoveGroupCommander('arm', wait_for_servers=60.0)
        self.group.set_max_velocity_scaling_factor(self.p['max_velocity_scale'])
        self.group.set_max_acceleration_scaling_factor(self.p['max_acceleration_scale'])
        self.group.set_goal_tolerance(0.01)
        self.group.set_planning_time(3.0)
        self.log('MoveIt move_group 已连接, 规划组: arm')

        # ---------- 等待控制系统就绪 ----------
        self._wait_for_system()

        # ---------- 规划场景: 桌面 + 目标物体 ----------
        self._publish_scene()
        time.sleep(1.5)  # 等 move_group 的场景监视器更新
        self.log('规划场景已添加: 桌面 + 目标物体')

    # ================= 参数与日志 =================

    def _load_params(self):
        params = dict(DEFAULT_PARAMS)
        path = self.get_parameter('params_file').value
        if not path:
            path = os.path.join(
                get_package_share_directory('robomaster_pick_place_sim'),
                'config', 'pick_place_params.yaml')
        if os.path.exists(path):
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            params.update(data.get('pick_place_node', {}).get('ros__parameters', {}))
            self.get_logger().info(f'参数文件: {path}')
        else:
            self.get_logger().warn(f'参数文件不存在, 使用默认值: {path}')
        return params

    def log(self, msg, level='info'):
        line = f'[{datetime.now().strftime("%H:%M:%S")}] {msg}'
        print(line, flush=True)
        with open(self.log_path, 'a') as f:
            f.write(line + '\n')
        getattr(self.get_logger(), level)(msg)

    def status(self, msg):
        self.log(f'状态: {msg}')
        self.status_pub.publish(String(data=msg))

    # ================= 系统就绪等待 =================

    def _js_cb(self, msg):
        self._last_js = msg

    def _wait_joint_states(self, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._last_js is not None:
                return self._last_js
            rclpy.spin_once(self, timeout_sec=0.2)
        raise TaskError('/joint_states 超时无消息')

    def _current_joint_states(self):
        """取最新几帧关节状态(含夹爪真实位置)."""
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)
        return self._last_js

    def _wait_for_system(self):
        # 1. 等 joint_state_broadcaster 发布关节状态
        self.log('等待 /joint_states ...')
        self._wait_joint_states(timeout=60.0)

        # 2. 等 arm_controller 的 FollowJointTrajectory Action 服务
        self.log('等待 /arm_controller/follow_joint_trajectory ...')
        act_cli = ActionClient(self, FollowJointTrajectory,
                               '/arm_controller/follow_joint_trajectory')
        if not act_cli.wait_for_server(timeout_sec=60.0):
            raise TaskError('arm_controller Action 服务超时未出现')

        # 3. 等三个控制器都进入 active 状态
        self.log('等待控制器进入 active ...')
        cli = self.create_client(ListControllers, '/controller_manager/list_controllers')
        if not cli.wait_for_service(timeout_sec=60.0):
            raise TaskError('controller_manager 服务超时未出现')
        deadline = time.time() + 90.0
        while time.time() < deadline:
            fut = cli.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            if fut.done() and fut.result() is not None:
                states = {c.name: c.state for c in fut.result().controller}
                want = ['joint_state_broadcaster', 'arm_controller', 'gripper_controller']
                if all(states.get(w) == 'active' for w in want):
                    self.log('所有控制器已激活')
                    return
            time.sleep(2.0)
        raise TaskError('控制器激活超时')

    # ================= 规划场景 (桌面 + 物体 + attach) =================

    def _box_obj(self, name, center, size, op=CollisionObject.ADD):
        obj = CollisionObject()
        obj.id = name
        obj.header.frame_id = 'world'
        obj.header.stamp = self.get_clock().now().to_msg()
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = list(size)
        obj.primitives = [prim]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = center
        pose.orientation.w = 1.0
        obj.primitive_poses = [pose]
        obj.operation = op
        return obj

    def _publish_scene(self):
        ps = PlanningScene()
        ps.is_diff = True
        t = self.p['table']
        pp = self.p['pick_point']
        ps.world.collision_objects = [
            self._box_obj('table', t['center'], t['size']),
            self._box_obj('target_object', [pp['x'], pp['y'], pp['z']],
                          [self.p['object_size']] * 3),
        ]
        self.scene_pub.publish(ps)

    def _attach_object(self, attach):
        """把目标物体附加/解除到夹爪基座 (仅影响 MoveIt 规划场景)."""
        ps = PlanningScene()
        ps.is_diff = True
        ps.robot_state.is_diff = True
        att = AttachedCollisionObject()
        att.link_name = 'gripper_base_link'
        att.touch_links = ['gripper_base_link', 'left_finger_link', 'right_finger_link']
        pp = self.p['pick_point']
        att.object = self._box_obj(
            'target_object', [pp['x'], pp['y'], pp['z']],
            [self.p['object_size']] * 3,
            CollisionObject.ADD if attach else CollisionObject.REMOVE)
        ps.robot_state.attached_collision_objects = [att]
        self.scene_pub.publish(ps)
        time.sleep(1.0)  # 等场景监视器更新
        self.log('物体已附加到夹爪' if attach else '物体已从夹爪解除')

    # ================= 运动原语 =================

    def _ee_pos(self):
        pose = self.group.get_current_pose().pose
        return [pose.position.x, pose.position.y, pose.position.z]

    def _finger_dir(self):
        """当前末端姿态下手指指向 (夹爪局部 +X 在世界系下的方向)."""
        q = self.group.get_current_pose().pose.orientation
        return (1.0 - 2.0 * (q.y ** 2 + q.z ** 2),
                2.0 * (q.x * q.y + q.w * q.z),
                2.0 * (q.x * q.z - q.w * q.y))

    def _check_limits(self, label):
        js = self._current_joint_states()
        if js is None:
            return
        for name, value in zip(js.name, js.position):
            if name in JOINT_LIMITS:
                lo, hi = JOINT_LIMITS[name]
                if not lo - 0.01 <= value <= hi + 0.01:
                    raise TaskError(f'{label}: 关节 {name} 超限 '
                                    f'({value:.3f} 超出 [{lo}, {hi}])')

    def go_to_pos(self, pos, label):
        """关节空间规划到指定末端位置 (位置 IK, 姿态自由)."""
        if not self.group.set_position_target([pos[0], pos[1], pos[2]]):
            raise TaskError(f'{label}: 位置目标无效 {pos}')
        ok, traj, _, err = self.group.plan()
        if not ok:
            self.group.clear_pose_targets()
            raise TaskError(f'{label}: 规划失败/无逆解 (error code={err.code}), '
                            f'目标 {pos} 不可达或与障碍碰撞')
        if not self.group.execute(traj, wait=True):
            raise TaskError(f'{label}: 轨迹执行失败')
        self.group.clear_pose_targets()
        self._check_limits(label)
        self._record(label, True)
        self.log(f'{label}: 到达 ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})')

    def go_cartesian(self, pos, label):
        """沿直线(笛卡尔路径)移动到指定位置, 保持当前姿态."""
        cur = self.group.get_current_pose().pose
        wp = Pose()
        wp.position.x, wp.position.y, wp.position.z = pos
        wp.orientation = cur.orientation
        traj, fraction = self.group.compute_cartesian_path([wp], 0.005, 0.0)
        if traj is None or fraction < 0.95:
            raise TaskError(f'{label}: 笛卡尔路径规划失败 (fraction={fraction:.2f})')
        if not self.group.execute(traj, wait=True):
            raise TaskError(f'{label}: 笛卡尔轨迹执行失败')
        self._check_limits(label)
        self._record(label, True)
        self.log(f'{label}: 直线到达 ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})')

    def grip(self, value, label):
        self.grip_pub.publish(Float64MultiArray(data=[float(value), float(value)]))
        time.sleep(1.5)
        self._record(label, True)
        self.log(f'{label}: 夹爪指令 {value:.3f} m')

    def go_home(self):
        if not self.group.set_named_target('home'):
            raise TaskError('回零: SRDF 中找不到 home 位姿')
        ok, traj, _, err = self.group.plan()
        if not ok:
            raise TaskError(f'回零: 规划失败 (error code={err.code})')
        if not self.group.execute(traj, wait=True):
            raise TaskError('回零: 轨迹执行失败')
        self._record('home', True)
        self.log('回零完成')

    # ================= 记录 =================

    def _record(self, label, success):
        row = [datetime.now().strftime('%H:%M:%S.%f')[:-3], self.cycle, label,
               '', '', '', '', '', '', '', '', int(success)]
        js = self._current_joint_states()
        if js is not None:
            vals = dict(zip(js.name, js.position))
            row[3] = round(vals.get('base_yaw_joint', float('nan')), 4)
            row[4] = round(vals.get('arm_lift_joint', float('nan')), 4)
            row[5] = round(vals.get('wrist_pitch_joint', float('nan')), 4)
            row[6] = round(vals.get('left_finger_joint', float('nan')), 4)
            row[7] = round(vals.get('right_finger_joint', float('nan')), 4)
        try:
            pose = self.group.get_current_pose().pose
            row[8] = round(pose.position.x, 4)
            row[9] = round(pose.position.y, 4)
            row[10] = round(pose.position.z, 4)
        except Exception:
            pass
        self.csv_w.writerow(row)
        self.csv_f.flush()

    # ================= 抓取循环 =================

    def _do_cycle(self, i):
        self.cycle = i
        pp = self.p['pick_point']
        bp = self.p['place_point']
        steps = []

        self.status(f'开始第 {i}/{self.p["num_cycles"]} 次抓取')

        self.go_home()
        steps.append('home')

        # --- 取物点 A: 先到预抓取点, 读实际手指方向, 再沿手指方向直线接近 ---
        pre = [pp['x'] - self.p['approach_dist'], pp['y'], pp['z']]
        self.go_to_pos(pre, f'cycle{i}_pre_pick')
        steps.append(f'cycle{i}_pre_pick')
        f = self._finger_dir()
        pre = [pp['x'] - f[0] * self.p['approach_dist'],
               pp['y'] - f[1] * self.p['approach_dist'],
               pp['z'] - f[2] * self.p['approach_dist']]
        if math.dist(pre, self._ee_pos()) > 0.02:
            self.go_to_pos(pre, f'cycle{i}_pre_pick_align')
            steps.append(f'cycle{i}_pre_pick_align')
        pick_pos = [pp['x'] - f[0] * self.p['finger_center_offset'],
                    pp['y'] - f[1] * self.p['finger_center_offset'],
                    pp['z'] - f[2] * self.p['finger_center_offset']]
        self.go_cartesian(pick_pos, f'cycle{i}_pick_approach')
        steps.append(f'cycle{i}_pick_approach')

        # --- 夹取 + attach ---
        self.grip(self.p['gripper_closed'], f'cycle{i}_grip_close')
        steps.append(f'cycle{i}_grip_close')
        if self.p['attach_object']:
            self._attach_object(True)

        # --- 抬升到安全高度 ---
        lift = self._ee_pos()
        lift[2] += self.p['lift_dist']
        self.go_cartesian(lift, f'cycle{i}_lift')
        steps.append(f'cycle{i}_lift')

        # --- 放置点 B: 同样的自适应接近 ---
        pre = [bp['x'] - self.p['approach_dist'], bp['y'], bp['z']]
        self.go_to_pos(pre, f'cycle{i}_pre_place')
        steps.append(f'cycle{i}_pre_place')
        f = self._finger_dir()
        pre = [bp['x'] - f[0] * self.p['approach_dist'],
               bp['y'] - f[1] * self.p['approach_dist'],
               bp['z'] - f[2] * self.p['approach_dist']]
        if math.dist(pre, self._ee_pos()) > 0.02:
            self.go_to_pos(pre, f'cycle{i}_pre_place_align')
            steps.append(f'cycle{i}_pre_place_align')
        place_pos = [bp['x'] - f[0] * self.p['finger_center_offset'],
                     bp['y'] - f[1] * self.p['finger_center_offset'],
                     bp['z'] - f[2] * self.p['finger_center_offset']]
        self.go_cartesian(place_pos, f'cycle{i}_place_approach')
        steps.append(f'cycle{i}_place_approach')

        # --- 释放 + detach + 抬离 ---
        if self.p['attach_object']:
            self._attach_object(False)
        self.grip(self.p['gripper_open'], f'cycle{i}_release')
        steps.append(f'cycle{i}_release')
        lift = self._ee_pos()
        lift[2] += self.p['lift_dist']
        self.go_cartesian(lift, f'cycle{i}_withdraw')
        steps.append(f'cycle{i}_withdraw')

        self.go_home()
        steps.append('home')
        self.status(f'第 {i}/{self.p["num_cycles"]} 次抓取成功')
        return steps

    def _stop_safe(self, reason):
        """失败后的安全处理: 急停 -> 尽力回零 -> 记录错误."""
        self.status(f'错误: {reason}, 急停')
        self.group.stop()
        self._record(f'stop:{reason}', False)
        try:
            self.go_home()
        except Exception:
            self.log('警告: 回零也失败, 机械臂已停止', 'error')

    def run(self):
        # 启动前可达性预检查: A/B 两点都规划不通就整体中止
        for name in ('pick_point', 'place_point'):
            pos = self.p[name]
            xyz = [pos['x'], pos['y'], pos['z']]
            if not self.group.set_position_target(xyz):
                self.log(f'错误: {name} {xyz} 位置目标无效, 任务中止', 'error')
                return 2
            ok, _, _, err = self.group.plan()
            self.group.clear_pose_targets()
            if not ok:
                self.log(f'错误: {name} {xyz} 不可达/无逆解 (error code={err.code}), '
                         f'任务中止, 请用 calibrate_pose 重新标定', 'error')
                self._stop_safe(f'{name} 不可达')
                return 1
            self.log(f'可达性检查通过: {name} {xyz}')

        self.status('开始连续抓取实验')
        for i in range(1, self.p['num_cycles'] + 1):
            t0 = time.time()
            try:
                steps = self._do_cycle(i)
                self.successes += 1
                self.cycle_records.append(
                    {'cycle': i, 'success': True, 'steps': steps,
                     'duration_s': round(time.time() - t0, 1), 'error': None})
            except TaskError as e:
                self.failures += 1
                self.log(f'第 {i} 次抓取失败: {e}', 'error')
                self.cycle_records.append(
                    {'cycle': i, 'success': False, 'steps': [],
                     'duration_s': round(time.time() - t0, 1), 'error': str(e)})
                self._stop_safe(str(e))
                if self.p['abort_on_error']:
                    self.status('检测到错误且 abort_on_error=true, 任务终止')
                    break
        return 0

    def finish(self):
        self.status(f'实验结束: 成功 {self.successes}/{self.p["num_cycles"]}, '
                    f'失败 {self.failures}')
        passed = self.successes >= max(1, int(0.8 * self.p['num_cycles']))
        result = {
            'experiment': 'robomaster_ep_pick_place',
            'finish_time': datetime.now().isoformat(),
            'num_cycles': self.p['num_cycles'],
            'successes': self.successes,
            'failures': self.failures,
            'passed': bool(passed),
            'cycles': self.cycle_records,
            'params': self.p,
            'log_file': self.log_path,
            'trajectory_file': self.csv_path,
        }
        with open(self.json_path, 'w') as f:
            json.dump(result, f, ensure_ascii=False, indent=2)
        self.csv_f.close()
        self.log(f'验收结果: {"通过" if passed else "未通过"} '
                 f'({self.successes}/{self.p["num_cycles"]} >= 80%)')
        self.log(f'结果已保存: {self.json_path}')
        self.log(f'轨迹已保存: {self.csv_path}')


def main():
    rclpy.init()
    node = PickPlaceNode()
    code = 0
    try:
        code = node.run()
    except TaskError as e:
        node.log(f'致命错误: {e}', 'error')
        code = 1
    except KeyboardInterrupt:
        node.log('用户中断, 执行安全停止', 'warn')
        try:
            node._stop_safe('用户中断')
        except Exception:
            pass
        code = 130
    finally:
        try:
            node.finish()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
