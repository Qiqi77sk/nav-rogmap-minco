# shengsai_bt

本包是 `shengsai2026` 地图比赛任务的独立决策行为树，不依赖也不修改 `bt_manager`。

## 任务流程

当前生效行为树只有三个业务节点：

```text
MoveToTarget
  禁止自瞄，姿态 MOVE
  等待 Nav2 action server 和新鲜 /sentry/offline_info
  从起点导航到终点，单次 120 s，最多 2 次重试
  Nav2 判定位置和 yaw 到达
  若 action 失败/超时，feedback 位姿在容差内也按到达处理
  → 立即向 /cmd_vel_mpc 发送零速，并保持零速

EngageAtTarget
  姿态 ATTACK，not_aim_enemy=false
  等待 is_get=true，锁存攻击开始
  再等待之后的 is_get=false，锁存攻击完成
  攻击期间以 10 Hz 保持 /cmd_vel_mpc 零速
  正常完成、等待攻击开始超时、攻击执行超时、offline_info 过期
  → 结束攻击，恢复 MOVE + 禁止自瞄，进入返航

ReturnToStart
  恢复 MOVE + 禁止自瞄
  清除零速保持，交还 MPC 控制
  从终点导航回起点，单次 120 s，最多 2 次重试
  Nav2 判定位置和 yaw 到达
  若 action 失败/超时，feedback 位姿在容差内也按到达处理
  → 立即向 /cmd_vel_mpc 发送零速并保持

任务终态
  SUCCESS/FAILURE 均锁存，不再 tick 行为树
  保持 MOVE + 禁止自瞄 + /cmd_vel_mpc 零速
  ROS 节点和 10 Hz 安全状态发布器继续运行
```

到点时的初始 `is_get=false` 不会被当作攻击结束；只有先收到 `is_get=true`，之后的 `is_get=false` 才表示攻击完成。

## 导航到达 fallback

`NavigateToGoal` 会读取最后一次 `NavigateToPose` feedback。满足以下条件时按到达处理：

- feedback 新鲜（默认 2 s 内）；
- feedback 位姿在 `map` 系；
- 距目标不超过 `arrival_xy_tolerance`（默认 0.20 m）；
- yaw 误差不超过 `arrival_yaw_tolerance`（默认 0.15 rad）。

触发方式有两种：

1. **action 仍在 RUNNING**：feedback 位姿连续 `arrival_dwell_s`（默认 0.5 s）在容差内，取消当前导航 goal，打印 WARN 并进入下一阶段；
2. **action 失败/超时**：立即检查 feedback，满足容差则打印 WARN 并按到达处理，不再重试或锁存失败。

该 fallback 只影响导航到达判定；正常 `SUCCEEDED` 不受影响。容差和 dwell 对应 XML 端口 `arrival_xy_tolerance`、`arrival_yaw_tolerance`、`feedback_stale_s`、`arrival_dwell_s`。

## 攻击零速

- 进入 `EngageAtTarget` 时立即发送一帧 `/cmd_vel_mpc` 零速。
- 攻击期间由 10 Hz `publishBehavior()` 持续发送 `/cmd_vel_mpc` 零速。
- `MoveToTarget` / `ReturnToStart` 成功时也会立即发送一帧零速，保证 communication 缓存中的最后一帧是 0。
- 每次开始新的导航尝试前清除零速保持，避免返航或重试时与 MPC 抢控制。
- `/cmd_vel` 的历史零速发布保留，但底盘实际使用的是 `/cmd_vel_mpc`。

## 接口

| 方向 | 接口 | 用途 |
|---|---|---|
| 输入 | `/sentry/offline_info` | 读取 `is_get`，true 表示下位机已进入击打姿态 |
| Action | `/navigate_to_pose` | 复用现有 Nav2、MiniCO Planner 和 MPC |
| 输出 | `/sentry/behaivor_send` | 10 Hz 发布 `Behavior` 消息 |
| 输出 | `/cmd_vel_mpc` | 攻击阶段和终态保持零速 |
| 输出 | `/cmd_vel` | 历史接口，终端停车时发布零速 |

`not_aim_enemy=true` 表示禁止自瞄。`MoveToTarget`、`ReturnToStart` 和终态使用 `MOVE + true`；`EngageAtTarget` 使用 `ATTACK + false`。

## 启动

先按原有方式启动 Point-LIO、GICP、Nav2 和 communication，再单独运行：

```bash
ros2 launch shengsai_bt shengsai_bt.launch.py
```

不要与旧 `bt_manager` 同时运行，否则两个节点会同时发布 `/sentry/behaivor_send`。

地图点、yaw、各阶段超时和重试次数都在 [config/shengsai.yaml](config/shengsai.yaml) 中配置。C++ 默认坐标仅作回退，当前生效值以 YAML 为准。
