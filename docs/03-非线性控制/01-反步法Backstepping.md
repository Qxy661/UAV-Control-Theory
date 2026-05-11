# 03-非线性控制：反步法 Backstepping

> 预计阅读：30 分钟 | 前置知识：Lyapunov 稳定性、状态空间控制、非线性系统基础

---

## 1. 反步法概述

反步法（Backstepping）是一种递归设计方法，适用于严格反馈形式的非线性系统。它通过逐步构造 Lyapunov 函数和虚拟控制律，最终得到实际控制输入。

### 1.1 核心思想

```
┌─────────────────────────────────────────────────┐
│              反步法核心思想                        │
├─────────────────────────────────────────────────┤
│                                                 │
│  严格反馈系统：                                   │
│  ẋ₁ = f₁(x₁) + g₁(x₁)x₂                       │
│  ẋ₂ = f₂(x₁,x₂) + g₂(x₁,x₂)x₃                │
│  ...                                            │
│  ẋₙ = fₙ(x₁,...,xₙ) + gₙ(x₁,...,xₙ)u          │
│                                                 │
│  设计思路（从后往前）：                            │
│  步骤1：设计虚拟控制 α₁ 稳定第一个子系统           │
│  步骤2：将 α₁ 视为"期望输入"，设计 α₂             │
│  步骤3：递归进行，直到得到实际控制 u               │
│                                                 │
│  每一步都构造 Lyapunov 函数，保证稳定性            │
│                                                 │
└─────────────────────────────────────────────────┘
```

### 1.2 适用条件

| 条件 | 说明 |
|------|------|
| 严格反馈形式 | 系统可表示为严格反馈结构 |
| 下三角结构 | 每个方程只依赖前面的状态 |
| 可反馈线性化 | 虚拟控制可使子系统稳定 |

---

## 2. 反步法设计步骤

### 2.1 基本步骤

**第一步**：考虑第一个子系统

$$\dot{x}_1 = f_1(x_1) + g_1(x_1)x_2$$

设计虚拟控制 $\alpha_1$ 使 $x_1$ 稳定：

$$V_1 = \frac{1}{2} x_1^2$$

$$\dot{V}_1 = x_1 (f_1 + g_1 \alpha_1) < 0$$

选择 $\alpha_1 = -\frac{1}{g_1}(f_1 + c_1 x_1)$，$c_1 > 0$

**第二步**：引入误差变量 $z_2 = x_2 - \alpha_1$

$$\dot{z}_2 = f_2 + g_2 x_3 - \dot{\alpha}_1$$

设计虚拟控制 $\alpha_2$：

$$V_2 = V_1 + \frac{1}{2} z_2^2$$

$$\dot{V}_2 = \dot{V}_1 + z_2 (f_2 + g_2 \alpha_2 - \dot{\alpha}_1)$$

**第 n 步**：得到实际控制 $u = \alpha_n$

### 2.2 MATLAB 实现

```matlab
function u = backstepping_controller(x, params)
    % 反步法控制器
    % x: 状态向量 [x1, x2, ..., xn]
    % params: 参数结构体

    c1 = params.c1;
    c2 = params.c2;
    % ...

    % 第一步：虚拟控制 α₁
    f1 = x(1)^2;  % 示例非线性函数
    g1 = 1;
    alpha1 = -(f1 + c1*x(1)) / g1;

    % 第二步：虚拟控制 α₂
    z2 = x(2) - alpha1;
    f2 = sin(x(1)) + x(2);  % 示例
    g2 = 1;
    dalpha1 = (2*x(1)*(x(1)^2) + c1*x(2));  % α₁的导数
    alpha2 = -(f2 - dalpha1 + c2*z2) / g2;

    % 最终控制
    u = alpha2;
end
```

---

## 3. 反步法在四旋翼中的应用

### 3.1 四旋翼姿态反步控制

**系统模型**：

$$\dot{\phi} = p + q \sin\phi \tan\theta + r \cos\phi \tan\theta$$
$$\dot{\theta} = q \cos\phi - r \sin\phi$$
$$\dot{\psi} = q \sin\phi / \cos\theta + r \cos\phi / \cos\theta$$

**反步法设计**：

```matlab
function tau = attitude_backstepping(phi, theta, psi, p, q, r, ...
    phi_ref, theta_ref, psi_ref, phi_ref_dot, theta_ref_dot, ...
    psi_ref_dot, params)
    % 四旋翼姿态反步控制（严格反步法）
    %
    % 步骤:
    %   1) 定义误差 z1 = phi - phi_ref
    %   2) 设计虚拟控制 alpha 使得 V1 = 0.5*z1^2 递减
    %   3) 定义 z2 = p - alpha，构造 V = 0.5*z1^2 + 0.5*z2^2
    %   4) 求解最终力矩使 dV/dt < 0

    c1 = params.c1;   % 第一层增益
    c2 = params.c2;   % 第二层增益

    % ========== 以滚转角 phi 通道为例 ==========
    % --- 第一步：定义跟踪误差 ---
    z1 = phi - phi_ref;

    % --- 第二步：虚拟控制律 alpha ---
    %   选取 V1 = 0.5 * z1^2
    %   令 dV1/dt = z1 * (p - phi_ref_dot) = -c1 * z1^2
    %   => alpha = phi_ref_dot - c1 * z1
    alpha = phi_ref_dot - c1 * z1;

    %   虚拟控制的导数（需要 phi_ref_ddot，此处用数值近似）
    alpha_dot = -c1 * (p - phi_ref_dot);  % d(alpha)/dt 的近似

    % --- 第三步：定义第二层误差 ---
    z2 = p - alpha;

    % --- 第四步：最终控制律 ---
    %   V = 0.5*z1^2 + 0.5*z2^2
    %   dV/dt = z1*z1_dot + z2*z2_dot
    %        = z1*(z2 + alpha - phi_ref_dot) + z2*(u - alpha_dot)
    %        = z1*(z2 - c1*z1) + z2*(u - alpha_dot)
    %   令 dV/dt = -c1*z1^2 - c2*z2^2
    %   => u = alpha_dot - z1 - c2*z2
    tau_phi = alpha_dot - z1 - c2 * z2;

    % ========== 俯仰角 theta 通道（同理） ==========
    z1_theta = theta - theta_ref;
    alpha_theta = theta_ref_dot - c1 * z1_theta;
    alpha_theta_dot = -c1 * (q - theta_ref_dot);
    z2_theta = q - alpha_theta;
    tau_theta = alpha_theta_dot - z1_theta - c2 * z2_theta;

    % ========== 偏航角 psi 通道（同理） ==========
    z1_psi = psi - psi_ref;
    alpha_psi = psi_ref_dot - c1 * z1_psi;
    alpha_psi_dot = -c1 * (r - psi_ref_dot);
    z2_psi = r - alpha_psi;
    tau_psi = alpha_psi_dot - z1_psi - c2 * z2_psi;

    tau = [tau_phi; tau_theta; tau_psi];
end
```

### 3.2 级联反步控制

```
┌─────────────────────────────────────────────────┐
│           四旋翼级联反步控制                       │
├─────────────────────────────────────────────────┤
│                                                 │
│  位置参考 ──→ [位置反步] ──→ 姿态参考             │
│                    │                            │
│                    ↓                            │
│              [姿态反步] ──→ 力矩                 │
│                    │                            │
│                    ↓                            │
│              [角速率反步] ──→ 电机控制量           │
│                                                 │
│  优点：                                         │
│  ├── Lyapunov 稳定性保证                         │
│  ├── 递归设计，结构清晰                          │
│  └── 可处理非线性耦合                            │
│                                                 │
└─────────────────────────────────────────────────┘
```

---

## 4. 反步法的改进

> **相关章节**：动态面控制（DSC）通过引入低通滤波器代替解析求导，解决了反步法的"项爆炸"问题，是反步法最重要的改进之一，详见 [动态面控制DSC](./03-动态面控制DSC.md)。

### 4.1 自适应反步法

**问题**：系统参数未知

**解决**：引入参数估计

```matlab
function [u, theta_hat] = adaptive_backstepping(x, theta_hat, params)
    % 自适应反步法
    % theta_hat: 参数估计

    gamma = params.gamma;  % 自适应增益
    c = params.c;

    % 虚拟控制
    alpha = -(f(x, theta_hat) + c*x(1));

    % 参数更新律
    theta_hat_dot = -gamma * x(1) * df_dtheta(x, theta_hat);
    theta_hat = theta_hat + theta_hat_dot * dt;

    % 控制律
    u = alpha;
end
```

### 4.2 鲁棒反步法

**问题**：存在不确定性和扰动

**解决**：增加鲁棒项

```matlab
function u = robust_backstepping(x, params)
    % 鲁棒反步法
    c = params.c;
    rho = params.rho;  % 扰动界

    % 标称控制
    alpha_nominal = -(f(x) + c*x(1));

    % 鲁棒项
    s = x(2) - alpha_nominal;
    u_robust = -rho * sign(s);

    % 总控制
    u = alpha_nominal + u_robust;
end
```

---

## 5. 反步法与其他方法对比

| 方法 | 优势 | 劣势 | 适用场景 |
|------|------|------|---------|
| 反步法 | Lyapunov 稳定，递归设计 | 需严格反馈形式 | 下三角系统 |
| 滑模控制 | 鲁棒性强 | 抖振问题 | 不确定性大 |
| 反馈线性化 | 精确线性化 | 需精确模型 | 可线性化系统 |
| MPC | 处理约束 | 计算量大 | 约束严格 |

---

## 思考题

1. 反步法为什么要求系统是严格反馈形式？如果系统不满足这个条件怎么办？
2. 反步法中的"虚拟控制"有什么物理意义？
3. 自适应反步法和标准反步法的区别是什么？各有什么优缺点？
4. 反步法在四旋翼姿态控制中如何处理耦合项？
5. 反步法和动态面控制（DSC）有什么关系？

<details>
<summary>参考答案</summary>

1. 严格反馈要求原因：(1) 递归设计需要下三角结构；(2) 每步只需处理一个状态和一个虚拟控制；(3) 不满足时可尝试动态面控制或反馈线性化。解决方法：(1) 坐标变换使系统变为严格反馈形式；(2) 使用 DSC 放松条件。

2. 虚拟控制意义：(1) 将高阶系统分解为多个低阶子系统；(2) 每个虚拟控制使对应子系统稳定；(3) 物理上可理解为"期望的中间状态"；(4) 例如：位置控制器的输出是"期望速度"。

3. 区别：(1) 标准反步法假设参数已知；(2) 自适应反步法在线估计未知参数；(3) 优点：可处理参数不确定性；(4) 缺点：设计更复杂，需要持续激励。

4. 处理耦合：(1) 将耦合项视为"扰动"在虚拟控制中补偿；(2) 或显式包含耦合项在设计中；(3) 例如：滚转通道中 q*sin(phi)*tan(theta) 项在虚拟控制中处理。

5. 关系：(1) DSC 是反步法的改进；(2) 反步法需要解析求导，导致"项爆炸"；(3) DSC 用低通滤波器代替解析求导；(4) DSC 设计更简单，但稳定性分析更复杂。

</details>
