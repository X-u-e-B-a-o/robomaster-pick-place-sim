#!/usr/bin/env python3
"""RoboMaster EP 真机标定工具 (点动 + 锚点采集).

用途:
  1. 手动点动机械臂 (moveto / move / recenter), 夹爪测试 (open / close)
  2. 把 A 取物点、B 放置点、回零点在真机空间的实际坐标 (mm) 保存为
     标定锚点 anchor1_real / anchor2_real / home_real, 写入
     ep_driver_params.yaml, 供 ep_arm_driver 计算仿真->真机仿射映射.

使用方法 (在 Jetson 上, 与 EP 热点直连):
  ros2 run robomaster_ep_driver calibrate_real --config <ep_driver_params.yaml>

交互命令:
  pos              读取当前机械臂位置 (SDK 10Hz 推送)
  moveto <x> <y>   绝对位置移动 [mm], 工作范围 x∈[0,220] y∈[0,150]
  move <dx> <dy>   相对位置移动 [mm]
  recenter         回中 (moveto(0, 0))
  open / close     夹爪张开 / 夹紧
  anchor1          把当前位置保存为标定锚点 1 (A 取物点)
  anchor2          把当前位置保存为标定锚点 2 (B 放置点)
  home             把当前位置保存为真机回零点
  show             显示当前锚点值
  save             把锚点值写入参数文件
  quit             退出

安全: 点动期间注意机械臂活动范围, 两人操作 (一人持急停电源).
"""
import argparse
import os
import sys
import time

import yaml


def connect(conn_type):
    from robomaster import robot
    r = robot.Robot()
    r.initialize(conn_type=conn_type)
    r.robotic_arm.sub_position(freq=10, callback=on_position)
    return r


POS = [None, None]  # 最新推送位置 (x, y) [mm]


def on_position(pos):
    POS[0], POS[1] = pos


def wait_action(act, timeout=8.0):
    """等待 SDK 动作完成, 返回是否成功."""
    try:
        if not act.wait_for_completed(timeout=timeout):
            print(f'!! 动作超时 ({timeout}s 未完成)')
            return False
        return act._state == 'completed'
    except Exception as e:  # noqa: BLE001
        print(f'!! 动作下发失败: {e}')
        return False


def load_yaml(path):
    if not os.path.exists(path):
        print(f'!! 参数文件不存在: {path} (将新建)')
        return {'ep_arm_driver': {'ros__parameters': {}}}
    with open(path) as f:
        return yaml.safe_load(f)


def save_yaml(path, data):
    with open(path, 'w') as f:
        yaml.safe_dump(data, f, default_flow_style=False,
                       allow_unicode=True, sort_keys=False)
    print(f'锚点已写入: {path}')


def main():
    ap = argparse.ArgumentParser(description='EP 真机标定工具')
    ap.add_argument('--config', default=None,
                    help='ep_driver_params.yaml 路径 (默认自动查找)')
    ap.add_argument('--conn-type', default='ap', choices=['ap', 'sta', 'rndis'])
    args = ap.parse_args()

    config_path = args.config
    if config_path is None:
        try:
            from ament_index_python.packages import get_package_share_directory
            config_path = os.path.join(
                get_package_share_directory('robomaster_ep_driver'),
                'config', 'ep_driver_params.yaml')
        except Exception:  # noqa: BLE001
            config_path = os.path.join('config', 'ep_driver_params.yaml')

    data = load_yaml(config_path)
    params = data.setdefault('ep_arm_driver', {}).setdefault(
        'ros__parameters', {})

    robot = connect(args.conn_type)
    print('EP 连接成功. 输入命令 (help 查看全部命令):')

    def show_pos():
        if POS[0] is None:
            print('尚未收到位置推送, 请稍候重试')
            return None
        print(f'当前位置: ({POS[0]:.0f}, {POS[1]:.0f}) mm')
        return POS[0], POS[1]

    try:
        while True:
            try:
                line = input('ep> ').strip()
            except EOFError:
                break
            if not line:
                continue
            parts = line.split()
            cmd = parts[0].lower()
            if cmd in ('quit', 'exit', 'q'):
                break
            elif cmd in ('help', 'h', '?'):
                print(__doc__)
            elif cmd == 'pos':
                show_pos()
            elif cmd == 'recenter':
                wait_action(robot.robotic_arm.recenter())
            elif cmd == 'moveto' and len(parts) == 3:
                try:
                    x, y = float(parts[1]), float(parts[2])
                except ValueError:
                    print('用法: moveto <x> <y> (mm)')
                    continue
                wait_action(robot.robotic_arm.moveto(x=x, y=y))
            elif cmd == 'move' and len(parts) == 3:
                try:
                    dx, dy = float(parts[1]), float(parts[2])
                except ValueError:
                    print('用法: move <dx> <dy> (mm)')
                    continue
                wait_action(robot.robotic_arm.move(x=dx, y=dy))
            elif cmd == 'open':
                robot.gripper.open(power=60)
            elif cmd == 'close':
                robot.gripper.close(power=40)
            elif cmd in ('anchor1', 'anchor2', 'home') and len(parts) == 1:
                pos = show_pos()
                if pos is None:
                    continue
                x, y = int(round(pos[0])), int(round(pos[1]))
                if not (0 <= x <= 220 and 0 <= y <= 150):
                    print(f'!! ({x}, {y}) 超出官方工作范围 [0,220]x[0,150], '
                          f'请确认位置读数是否可信')
                if cmd == 'anchor1':
                    params['anchor1_real'] = [x, y]
                    print(f'anchor1_real (A 取物点) = [{x}, {y}]')
                elif cmd == 'anchor2':
                    params['anchor2_real'] = [x, y]
                    print(f'anchor2_real (B 放置点) = [{x}, {y}]')
                else:
                    params['home_real'] = [x, y]
                    print(f'home_real (回零点) = [{x}, {y}]')
            elif cmd == 'show':
                for k in ('anchor1_sim', 'anchor1_real', 'anchor2_sim',
                          'anchor2_real', 'home_real'):
                    print(f'  {k} = {params.get(k, "<未设置>")}')
            elif cmd == 'save':
                save_yaml(config_path, data)
            else:
                print('未知命令 (help 查看帮助)')
    finally:
        robot.close()


if __name__ == '__main__':
    main()
