# shengsai_bt

本包是 `shengsai2026` 地图比赛任务的独立决策行为树，不依赖也不修改 `bt_manager`。

## 任务流程

```text
安全等待
→ 前往蓝点，位置和 yaw 同时满足
→ 到点后立即 not_aim_enemy=false，进入 ATTACK
→ 等待视觉给出 is_get=true
→ 首次收到 is_get=false
→ not_aim_enemy=true
→ 返回红点，位置和 yaw 同时满足
→ 停车并保持安全输出
```

任意导航、等待目标、交战或通信超时都会取消当前导航、停车并输出失败日志。

## 接口

| 方向 | 接口 | 用途 |
|---|---|---|
| 输入 | `/sentry/offline_info` | 读取 `is_get`，true 表示视觉已瞄准到敌人，false 表示未瞄准 |
| Action | `/navigate_to_pose` | 复用现有 Nav2、MiniCO Planner 和 MPC |
| 输出 | `/sentry/behaivor_send` | 使用原通信包，10 Hz 发布现有 `Behavior` 消息 |
| 输出 | `/cmd_vel` | 仅在任务完成或失败后发布零速度停车 |

`not_aim_enemy=true` 表示禁止自瞄；到达蓝点后立即设为 false 并进入 ATTACK，等待视觉确认 `is_get=true`，首次收到 `is_get=false` 后立即恢复为 true 并锁存。

## 启动

先按原有方式启动 Point-LIO、GICP、Nav2 和 communication，再单独运行：

```bash
ros2 launch shengsai_bt shengsai_bt.launch.py
```

不要与旧 `bt_manager` 同时运行，否则两个节点会同时发布 `/sentry/behaivor_send`。

地图点、yaw 和各阶段超时时间都在 [config/shengsai.yaml](config/shengsai.yaml) 中配置。
