# GitHub 仓库注释

> 收录与 UAV 控制相关的 GitHub 仓库，按主题分类。

---

## 1. 开源飞控平台

| 仓库 | 链接 | 说明 | 推荐度 |
|------|------|------|--------|
| PX4-Autopilot | https://github.com/PX4/PX4-Autopilot | 最流行的开源飞控固件 | ⭐⭐⭐⭐⭐ |
| ArduPilot | https://github.com/ArduPilot/ardupilot | 功能强大的开源飞控 | ⭐⭐⭐⭐⭐ |
| Betaflight | https://github.com/betaflight/betaflight | 竞速无人机飞控 | ⭐⭐⭐⭐ |
| Cleanflight | https://github.com/cleanflight/cleanflight | 早期开源飞控 | ⭐⭐⭐ |

**PX4-Autopilot 重点目录**：
```
PX4-Autopilot/
├── src/modules/mc_att_control/    # 姿态控制器
├── src/modules/mc_pos_control/    # 位置控制器
├── src/modules/ekf2/              # EKF 状态估计
└── src/modules/sensors/           # 传感器驱动
```

**ArduPilot 重点目录**：
```
ardupilot/
├── ArduCopter/                    # 四旋翼控制
├── libraries/AC_AttitudeControl/  # 姿态控制库
├── libraries/AC_PID/              # PID 控制库
└── libraries/AP_Math/             # 数学工具库
```

---

## 2. 控制理论工具

| 仓库 | 链接 | 说明 | 推荐度 |
|------|------|------|--------|
| control-toolbox | https://github.com/ethz-adrl/control-toolbox | 控制工具箱 | ⭐⭐⭐⭐⭐ |
| python-control | https://github.com/python-control/python-control | Python 控制库 | ⭐⭐⭐⭐⭐ |
| casadi | https://github.com/casadi/casadi | 非线性优化 | ⭐⭐⭐⭐⭐ |
| do-mpc | https://github.com/do-mpc/do-mpc | MPC 框架 | ⭐⭐⭐⭐ |
| MPC-Toolkit | https://github.com/FilippoAiraldi/MPC-Toolkit | MPC 工具 | ⭐⭐⭐ |

**python-control 示例**：
```python
import control
import numpy as np

# 创建传递函数
sys = control.tf([1], [1, 2, 1])

# 绘制 Bode 图
control.bode_plot(sys)

# 绘制根轨迹
control.root_locus(sys)

# LQR 设计
A = np.array([[0, 1], [0, 0]])
B = np.array([[0], [1]])
Q = np.eye(2)
R = np.array([[1]])
K, S, E = control.lqr(A, B, Q, R)
```

**CasADi 示例**：
```python
import casadi as ca

# 定义优化变量
opti = ca.Opti()
x = opti.variable(2)
u = opti.variable(1)

# 目标函数
opti.minimize(x[0]**2 + x[1]**2 + u**2)

# 约束
opti.subject_to(x[1] == x[0] + u)

# 求解
opti.solver('ipopt')
sol = opti.solve()
```

---

## 3. 仿真环境

| 仓库 | 链接 | 说明 | 推荐度 |
|------|------|------|--------|
| AirSim | https://github.com/microsoft/AirSim | 微软仿真平台 | ⭐⭐⭐⭐⭐ |
| Gazebo | https://github.com/osrf/gazebo | ROS 仿真器 | ⭐⭐⭐⭐⭐ |
| PyBullet | https://github.com/bulletphysics/bullet3 | 物理仿真 | ⭐⭐⭐⭐ |
| Isaac Gym | https://github.com/NVIDIA-Omniverse/IsaacGymEnvs | GPU 仿真 | ⭐⭐⭐⭐ |

**AirSim 示例**：
```python
import airsim

# 连接仿真器
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

# 起飞
client.takeoffAsync().join()

# 移动到目标位置
client.moveToPositionAsync(10, 10, -5, 5).join()

# 获取状态
state = client.getMultirotorState()
print(state.kinematics_estimated.position)
```

---

## 4. 强化学习框架

| 仓库 | 链接 | 说明 | 推荐度 |
|------|------|------|--------|
| stable-baselines3 | https://github.com/DLR-RM/stable-baselines3 | RL 算法库 | ⭐⭐⭐⭐⭐ |
| CleanRL | https://github.com/vwxyzjn/cleanrl | 简洁 RL 实现 | ⭐⭐⭐⭐ |
| RLlib | https://github.com/ray-project/ray | 分布式 RL | ⭐⭐⭐⭐ |
| Gym-Pybullet-drones | https://github.com/utiasDSL/gym-pybullet-drones | 无人机 RL 环境 | ⭐⭐⭐⭐⭐ |

**stable-baselines3 示例**：
```python
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env

# 创建环境
env = make_vec_env("CartPole-v1", n_envs=4)

# 创建 Agent
model = PPO("MlpPolicy", env, verbose=1)

# 训练
model.learn(total_timesteps=100000)

# 保存模型
model.save("ppo_cartpole")

# 测试
obs = env.reset()
for _ in range(1000):
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
```

---

## 5. 无人机专用工具

| 仓库 | 链接 | 说明 | 推荐度 |
|------|------|------|--------|
| dronekit-python | https://github.com/dronekit/dronekit-python | 无人机编程接口 | ⭐⭐⭐⭐ |
| MAVSDK | https://github.com/mavlink/MAVSDK | MAVLink SDK | ⭐⭐⭐⭐⭐ |
| pymavlink | https://github.com/ArduPilot/pymavlink | MAVLink Python 库 | ⭐⭐⭐⭐ |
| Rotors Simulator | https://github.com/ethz-asl/rotors_simulator | 旋翼仿真 | ⭐⭐⭐⭐ |

**MAVSDK 示例**：
```python
from mavsdk import System
import asyncio

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    # 起飞
    await drone.action.takeoff()
    await asyncio.sleep(5)

    # 移动到目标位置
    await drone.action.goto_location(47.398039, 8.545572, 500, 0)
    await asyncio.sleep(10)

    # 着陆
    await drone.action.land()

asyncio.run(run())
```

---

## 6. 数学与优化工具

| 仓库 | 链接 | 说明 | 推荐度 |
|------|------|------|--------|
| scipy | https://github.com/scipy/scipy | 科学计算 | ⭐⭐⭐⭐⭐ |
| numpy | https://github.com/numpy/numpy | 数值计算 | ⭐⭐⭐⭐⭐ |
| cvxpy | https://github.com/cvxpy/cvxpy | 凸优化 | ⭐⭐⭐⭐⭐ |
| osqp | https://github.com/osqp/osqp | QP 求解器 | ⭐⭐⭐⭐⭐ |

**cvxpy 示例**：
```python
import cvxpy as cp
import numpy as np

# 定义优化变量
x = cp.Variable(2)

# 目标函数
objective = cp.Minimize(cp.sum_squares(x) + cp.norm(x, 1))

# 约束
constraints = [x >= -1, x <= 1]

# 求解
prob = cp.Problem(objective, constraints)
prob.solve()

print("最优值:", prob.value)
print("最优解:", x.value)
```

---

## 7. 学习资源仓库

| 仓库 | 链接 | 说明 | 推荐度 |
|------|------|------|--------|
| Awesome-UAV | https://github.com/liguge/Awesome-UAV | UAV 资源汇总 | ⭐⭐⭐⭐⭐ |
| Awesome-RL | https://github.com/jbhuang0604/awesome-rl | RL 资源汇总 | ⭐⭐⭐⭐ |
| Control-Theory | https://github.com/OverLordGoldDragon/control-theory | 控制理论笔记 | ⭐⭐⭐⭐ |
| MATLAB-robotics | https://github.com/petercorke/robotics-toolbox-matlab | 机器人工具箱 | ⭐⭐⭐⭐ |

---

## 8. 推荐学习路径

```
初学者：
├── python-control（控制理论基础）
├── PX4-Autopilot（飞控架构理解）
└── gym-pybullet-drones（RL 入门）

进阶者：
├── casadi（非线性优化）
├── do-mpc（MPC 实现）
└── stable-baselines3（RL 算法）

研究者：
├── control-toolbox（高级控制）
├── AirSim/Gazebo（高保真仿真）
└── cvxpy（凸优化）
```
