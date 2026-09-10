# RoboMaster EP 真机定点抓取 — 实验运行手册

真机阶段复用仿真阶段的全部 ROS 2 接口与任务逻辑, 仅更换设备层
(ep_arm_driver 伪装 ros2_control 接口) 与参数 (点位/速度/回零/标定锚点),
满足实验要求 "只修改设备、通信和位置参数, 不重写任务逻辑"。

## 1. 场地布置

- 机械臂基座固定, 目标物为边长 7cm、质量 ≤100g 的立方体。
- **A 取物点**在机械臂正前方 `(60, 40) mm` 附近, **B 放置点**在 A 正后方
  同一条线上 `(170, 60) mm` 附近 (EP 臂无偏航轴, 两点必须一前一后)。
  具体以现场标定值为准 (见 §4)。
- 桌面高度与臂基座齐平, 机械臂活动范围内无遮挡。
- 急停电源排插放在监守人员手边。

## 2. 连接与部署 (Jetson 10.140.246.156)

```bash
# 2.1 EP 开机, 热点直连 (conn_type='ap'), Jetson 加入 EP WiFi
# 2.2 部署代码 (本机仓库 robomaster-pick-place-sim, 分支 real-ep-arm)
rsync -av --delete --exclude 'results*' --exclude '.git' \
  robomaster-pick-place-sim/ <user>@10.140.246.156:~/ep_ws/src/robomaster-pick-place-sim/
ssh <user>@10.140.246.156

# 2.3 安装官方 SDK (PyPI wheel 仅 x86_64, aarch64 必须从源码装;
#     依赖 numpy/opencv-python/netaddr/netifaces/myqr 由 pip 自动装)
mkdir -p ~/ep_ws/src/third_party && cd ~/ep_ws/src/third_party
git clone https://github.com/dji-sdk/RoboMaster-SDK.git
pip3 install --user ./RoboMaster-SDK

# 2.4 打 libmedia_codec 存根 (SDK media.py 在模块级 import 视频解码库,
#     aarch64 无对应 .so; 本实验无视觉需求, 空模块即可满足导入)
mkdir -p ~/ep_ws/stubs
printf '# RoboMaster SDK libmedia_codec 存根: 本实验不使用视频功能\n' \
  > ~/ep_ws/stubs/libmedia_codec.py
echo 'export PYTHONPATH=$HOME/ep_ws/stubs:$PYTHONPATH' >> ~/.bashrc
source ~/.bashrc

# 2.5 构建
cd ~/ep_ws && colcon build --symlink-install && source install/setup.bash
```

## 3. 标定 (必须一次, 之后参数文件直接复用)

```bash
ros2 run robomaster_ep_driver calibrate_real
#   moveto <x> <y> / move <dx> <dy>  点动到 A 取物点正上方 (夹爪能抓住物体)
#   anchor1                           保存 A 取物点 -> anchor1_real
#   点动到 B 放置点 -> anchor2         保存 B 放置点 -> anchor2_real
#   点动到回零位 -> home               保存回零点 -> home_real
#   save                              写入 config/ep_driver_params.yaml
```

标定后确认 `config/ep_driver_params.yaml` 中 anchor1_real/anchor2_real/
home_real 与现场一致。

## 4. 干跑检查 (不控真机, 先验证整条链路)

```bash
ros2 launch robomaster_ep_driver pick_place_real.launch.py dry_run:=true
```

观察要点:
- `ep_arm_driver` 打印标定映射系数与全部 `[dry_run] moveto(x, y)` 路径;
- 所有真机坐标落在 `[0,220]x[0,150] mm` 内 (越界会 INVALID_GOAL);
- `pick_place_moveit` 预检查通过 (可达性 + 7 段路径预演), 5 次循环跑完;
- 日志/轨迹/结果写入 `results_real/`。

## 5. 正式运行 (低速首跑 + 双人操作)

```bash
ros2 launch robomaster_ep_driver pick_place_real.launch.py
```

- **首跑低速**: `pick_place_params_real.yaml` 中 `max_velocity_scale: 0.2`;
- **两人操作**: 一人看终端/日志, 一人专职急停 (手放断电排插开关);
- 遇异常立即断电 -> 终端 Ctrl+C -> 检查日志 `results_real/run_*.log`;
- 任务节点自身失败路径: 急停事件 -> 尽力回零 -> 记录错误继续下一次,
  连续 5 次抓取 ≥4 次成功即验收通过。

## 6. 急停手段

| 手段 | 触发方式 | 效果 |
|---|---|---|
| 硬件急停 | 断电排插开关 (监守人) | 立即停臂, 首选 |
| 软件冻结 | `ros2 service call /ep_arm/freeze std_srvs/srv/SetBool "{data: true}"` | 停止下发新指令, 当前动作完成后保持位置 |
| 任务急停 | 任务节点错误处理 / move_group stop 事件 | 取消当前轨迹, 尽力回零 |

## 7. 结果留存 (演示视频素材)

- `results_real/run_YYYYmmdd_HHMMSS.log` — 全流程日志
- `results_real/run_*_trajectory.csv` — 每个节点的时间/关节/末端位姿轨迹
- `results_real/run_*_results.json` — 5 次循环成败与验收结论
- 演示视频: 侧方机位拍摄 5 次连续抓取全流程, 含一次异常急停演示
