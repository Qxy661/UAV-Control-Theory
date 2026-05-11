# 02-现代控制方法：自抗扰控制 ADRC

> 预计阅读：30 分钟 | 前置知识：PID控制、状态空间控制、观测器设计

---

## 1. ADRC 概述

自抗扰控制（Active Disturbance Rejection Control, ADRC）是由韩京清研究员提出的一种不依赖精确模型的控制方法。其核心思想是：**将系统内部不确定性和外部扰动统一视为"总扰动"，通过扩张状态观测器实时估计并补偿**。

### 1.1 ADRC 的核心思想

```
┌─────────────────────────────────────────────────┐
│              ADRC 核心思想                        │
├─────────────────────────────────────────────────┤
│                                                 │
│  传统 PID 的问题：                               │
│  ├── 依赖误差的过去信息（积分）                   │
│  ├── 对扰动被动响应                              │
│  └── 无法区分不同类型的不确定性                   │
│                                                 │
│  ADRC 的解决：                                   │
│  ├── ESO 实时估计"总扰动"                        │
│  ├── 前馈补偿扰动，变被动为主动                   │
│  └── 不需要精确模型                              │
│                                                 │
│  总扰动 = 内部不确定性 + 外部扰动                 │
│  估计 + 补偿 = 将不确定系统化为积分串联型          │
│                                                 │
└─────────────────────────────────────────────────┘
```

### 1.2 ADRC 发展历程

| 年份 | 里程碑 | 贡献者 |
|------|--------|--------|
| 1998 | 提出 ADRC 概念 | 韩京清 |
| 2002 | 扩张状态观测器 (ESO) 完善 | 韩京清 |
| 2006 | 线性 ADRC (LADRC) | 高志强 |
| 2010s | ADRC 理论体系完善 | 多位学者 |
| 2020s | ADRC 在工业界广泛应用 | 工程实践 |

---

## 2. ADRC 组成部分

### 2.1 跟踪微分器（Tracking Differentiator, TD）

**作用**：为参考信号安排过渡过程，提取微分信号。

**离散形式**：

$$\begin{cases}
x_1(k+1) = x_1(k) + h \cdot x_2(k) \\
x_2(k+1) = x_2(k) + h \cdot \text{fhan}(x_1(k) - v(k), x_2(k), r, h_0)
\end{cases}$$

其中 $\text{fhan}$ 为最速综合函数：

$$\text{fhan}(x_1, x_2, r, h_0) = \begin{cases}
-d = r \cdot h_0 \\
a_0 = h_0 \cdot x_2 \\
y = x_1 + a_0 \\
a_1 = \sqrt{d(d + 8|y|)} \\
a_2 = a_0 + \text{sign}(y)(a_1 - d)/2 \\
s_y = (\text{sign}(y+d) - \text{sign}(y-d))/2 \\
a = (a_0 + y - a_2)s_y + a_2 \\
s_a = (\text{sign}(a+d) - \text{sign}(a-d))/2 \\
\text{fhan} = -r(a/d - \text{sign}(a))s_a - r \cdot \text{sign}(a)
\end{cases}$$

**MATLAB 实现**：

```matlab
function [x1, x2] = TD(v, x1_prev, x2_prev, r, h, h0)
    % 跟踪微分器
    % v: 参考输入
    % x1: 跟踪信号
    % x2: 微分信号
    % r: 速度因子
    % h: 采样步长
    % h0: 滤波因子

    d = r * h0;
    d0 = d * h0;
    y = x1_prev - v + h0 * x2_prev;

    a0 = sqrt(d * (d + 8 * abs(y)));
    a2 = x2_prev + (y - d0) / (2 * h0);

    if abs(y) > d0
        a = x2_prev + (a0 - d) / 2 * sign(y);
    else
        a = x2_prev + y / h0;
    end

    if abs(a) > d
        s = sign(a);
    else
        s = a / d;
    end

    fhan = -r * (a / d - s) * d - r * sign(a);

    x1 = x1_prev + h * x2_prev;
    x2 = x2_prev + h * fhan;
end
```

### 2.2 扩张状态观测器（Extended State Observer, ESO）

> **延伸阅读**：ESO 属于非线性观测器的一种，更多非线性观测器设计方法（高增益观测器、扩展观测器等）详见 [非线性观测器](../03-非线性控制/05-非线性观测器.md)。

**核心思想**：将系统总扰动扩展为一个新的状态，通过观测器估计。

**二阶系统**：$\ddot{y} = f(y, \dot{y}, w, t) + bu$

其中 $f$ 为总扰动（包括内部不确定性和外部扰动），$b$ 为控制增益。

**ESO 方程**：

$$\begin{cases}
\dot{z}_1 = z_2 - \beta_1 (z_1 - y) \\
\dot{z}_2 = z_3 - \beta_2 (z_1 - y) + b_0 u \\
\dot{z}_3 = -\beta_3 (z_1 - y)
\end{cases}$$

其中：
- $z_1 \approx y$：输出估计
- $z_2 \approx \dot{y}$：速度估计
- $z_3 \approx f$：总扰动估计
- $\beta_1, \beta_2, \beta_3$：观测器增益
- $b_0$：控制增益估计值

**线性 ESO（LESO）**：

$$\begin{cases}
\dot{z}_1 = z_2 - \omega_o(z_1 - y) \\
\dot{z}_2 = z_3 - \omega_o^2(z_1 - y) + b_0 u \\
\dot{z}_3 = -\omega_o^3(z_1 - y)
\end{cases}$$

其中 $\omega_o$ 为观测器带宽。

**MATLAB 实现**：

```matlab
function [z1, z2, z3] = LESO(y, u, z1_prev, z2_prev, z3_prev, b0, wo, dt)
    % 线性扩张状态观测器
    % y: 测量输出
    % u: 控制输入
    % b0: 控制增益估计
    % wo: 观测器带宽
    % dt: 采样步长

    e = z1_prev - y;

    z1_dot = z2_prev - 2*wo*e;
    z2_dot = z3_prev - wo^2*e + b0*u;
    z3_dot = -wo^3*e;

    z1 = z1_prev + z1_dot * dt;
    z2 = z2_prev + z2_dot * dt;
    z3 = z3_prev + z3_dot * dt;
end
```

### 2.3 非线性状态误差反馈（NLSEF）

**NLSEF 控制律**：

$$u_0 = k_1 \cdot \text{fal}(e_1, \alpha_1, \delta_1) + k_2 \cdot \text{fal}(e_2, \alpha_2, \delta_2)$$

$$u = \frac{u_0 - z_3}{b_0}$$

其中 $\text{fal}$ 为非线性函数：

$$\text{fal}(e, \alpha, \delta) = \begin{cases}
|e|^\alpha \cdot \text{sign}(e), & |e| > \delta \\
\frac{e}{\delta^{1-\alpha}}, & |e| \leq \delta
\end{cases}$$

**线性状态误差反馈（LSEF）**：

$$u_0 = k_1(r_1 - z_1) + k_2(r_2 - z_2)$$

$$u = \frac{u_0 - z_3}{b_0}$$

---

## 3. 完整 ADRC 结构

### 3.1 结构框图

```
┌─────────────────────────────────────────────────────────────┐
│                        ADRC 结构                             │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  r(t) ──→ [TD] ──→ r₁, r₂                                  │
│              │      │    │                                  │
│              │      ↓    ↓                                  │
│              │   ┌────────────┐                             │
│              │   │  NLSEF /   │                             │
│              │   │   LSEF     │──→ u₀                      │
│              │   └────────────┘    │                        │
│              │        ↑            ↓                        │
│              │        │      ┌──────────┐                   │
│              │        │      │ 补偿扰动 │                   │
│              │        │      │ u=(u₀-z₃)/b₀                │
│              │        │      └──────────┘                   │
│              │        │            │                        │
│              │        │            ↓                        │
│              │        │      ┌──────────┐                   │
│              │        │      │  被控对象 │──→ y              │
│              │        │      └──────────┘    │              │
│              │        │            ↑        │              │
│              │        │            │        │              │
│              │        └────────────┴────────┘              │
│              │                ↑                             │
│              │         ┌──────────┐                        │
│              └────────→│   ESO    │                        │
│                        │ z₁,z₂,z₃│                        │
│                        └──────────┘                        │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 3.2 MATLAB 完整实现

```matlab
function u = ADRC_controller(r, y, state, params)
    % ADRC 控制器完整实现
    % r: 参考输入
    % y: 测量输出
    % state: 控制器状态 [x1_td, x2_td, z1, z2, z3]
    % params: 参数结构体

    % 参数
    r_td = params.r_td;      % TD 速度因子
    h = params.h;            % 采样步长
    h0 = params.h0;          % TD 滤波因子
    wo = params.wo;          % ESO 带宽
    b0 = params.b0;          % 控制增益估计
    k1 = params.k1;          % 反馈增益1
    k2 = params.k2;          % 反馈增益2

    % 提取状态
    x1_td = state(1);  x2_td = state(2);
    z1 = state(3);     z2 = state(4);    z3 = state(5);

    % 1. 跟踪微分器
    [x1_td_new, x2_td_new] = TD(r, x1_td, x2_td, r_td, h, h0);

    % 2. 计算控制量（LSEF）
    e1 = x1_td_new - z1;
    e2 = x2_td_new - z2;
    u0 = k1 * e1 + k2 * e2;

    % 3. 扰动补偿
    u = (u0 - z3) / b0;

    % 4. 更新 ESO（使用上一步的 u）
    [z1_new, z2_new, z3_new] = LESO(y, u, z1, z2, z3, b0, wo, h);

    % 更新状态
    state = [x1_td_new, x2_td_new, z1_new, z2_new, z3_new];
end
```

**MATLAB 示例：简化 ADRC 控制器函数**

```matlab
function u = ADRC_controller(z_ref, z, z_dot, z_hat, z_dot_hat, b0, omega_c, omega_o)
    % Tracking Differentiator (simplified)
    e = z_hat - z;
    % Extended State Observer (already estimated z_hat, z_dot_hat, f_hat)
    f_hat = z_dot_hat;  % Total disturbance estimate
    % Nonlinear state error feedback
    e1 = z_ref - z_hat;
    e2 = -z_dot_hat;
    Kp = omega_c^2;
    Kd = 2*omega_c;
    u0 = Kp*e1 + Kd*e2;
    u = (u0 - f_hat) / b0;
end
```

---

## 4. ADRC 参数整定

### 4.1 参数整定方法

**LESO 参数整定**：

ESO 带宽 $\omega_o$ 的选择：
- $\omega_o$ 越大：估计越快，但对噪声敏感
- $\omega_o$ 越小：估计越慢，但对噪声鲁棒
- 经验：$\omega_o \approx (3 \sim 10) \times \omega_c$（控制器带宽）

**控制器参数整定**：

```matlab
% ADRC 参数整定指南
% 步骤1：确定控制增益估计 b0
b0 = 1 / m;  % 对于质量 m 的系统

% 步骤2：确定 ESO 带宽 wo
wo = 5 * wc;  % wc 为期望控制器带宽

% 步骤3：确定控制器增益
wc = 10;  % 期望带宽
k1 = wc^2;
k2 = 2 * wc;

% 步骤4：TD 参数
r_td = 100;  % 速度因子，越大跟踪越快
h0 = 5 * h;  % 滤波因子
```

### 4.2 带宽法参数整定

**高志强提出的带宽法**：

对于二阶系统 $\ddot{y} = f + bu$：

| 参数 | 公式 | 说明 |
|------|------|------|
| $\omega_o$ | 观测器带宽 | 估计速度 |
| $\omega_c$ | 控制器带宽 | 响应速度 |
| $k_1$ | $\omega_c^2$ | 比例增益 |
| $k_2$ | $2\omega_c$ | 微分增益 |
| $\beta_1$ | $3\omega_o$ | ESO 增益1 |
| $\beta_2$ | $3\omega_o^2$ | ESO 增益2 |
| $\beta_3$ | $\omega_o^3$ | ESO 增益3 |

### 4.3 参数对性能的影响

| 参数 | 增大的效果 | 减小的效果 |
|------|-----------|-----------|
| $\omega_o$ | 估计更快，噪声增大 | 估计更慢，噪声减小 |
| $\omega_c$ | 响应更快，可能振荡 | 响应更慢，更平稳 |
| $b_0$ | 控制量减小 | 控制量增大 |

---

## 5. ADRC vs PID 对比

### 5.1 理论对比

| 方面 | PID | ADRC |
|------|-----|------|
| 模型依赖 | 不依赖 | 不依赖 |
| 扰动处理 | 被动（积分） | 主动（ESO 估计补偿） |
| 微分实现 | 直接微分（噪声大） | TD 或 ESO（更平滑） |
| 非线性 | 线性 | 可引入非线性 |
| 参数数量 | 3 个 | 5-8 个 |
| 调参难度 | 简单 | 中等 |
| 抗扰能力 | 有限 | 强 |

### 5.2 仿真对比

```matlab
% ADRC vs PID 仿真对比
% 被控对象：二阶系统 + 扰动
% G(s) = 1 / (s^2 + 2s + 1)

% PID 控制器
Kp = 10; Ki = 5; Kd = 2;
C_pid = pid(Kp, Ki, Kd);

% ADRC 参数
params.r_td = 100;
params.h = 0.001;
params.h0 = 0.005;
params.wo = 50;
params.b0 = 1;
params.k1 = 100;
params.k2 = 20;

% 仿真
t = 0:0.001:5;

% 阶跃响应
[y_pid, t_pid] = step(feedback(C_pid*G, 1), t);
[y_adrc, t_adrc] = sim_adrc(G, params, t);

% 阶跃扰动响应
% 在 t=2s 施加阶跃扰动 d=1
```

### 5.3 性能对比

| 性能指标 | PID | ADRC | 说明 |
|---------|-----|------|------|
| 上升时间 | 中 | 快 | ADRC 响应更快 |
| 超调量 | 中 | 小 | ADRC TD 减小超调 |
| 扰动抑制 | 慢 | 快 | ADRC 主动补偿 |
| 稳态误差 | 有（无积分） | 无 | ESO 估计常值扰动 |
| 噪声敏感 | 中 | 可调 | 取决于 ESO 带宽 |

---

## 6. ADRC 在无人机中的应用

### 6.1 四旋翼姿态 ADRC 控制

```matlab
% 四旋翼滚转通道 ADRC 控制
% 系统：Jx * φ_ddot = τ_φ + d
% 其中 d 为总扰动（包括气动扰动、模型不确定性）

% ADRC 参数
params.b0 = 1 / Jx;  % 控制增益估计
params.wo = 50;       % ESO 带宽
params.k1 = 100;      % 比例增益
params.k2 = 20;       % 微分增益

% ESO 状态
z1 = 0; z2 = 0; z3 = 0;

% 控制循环
for k = 1:length(t)
    % 测量
    y = phi_meas(k);

    % ESO 更新
    [z1, z2, z3] = LESO(y, u_prev, z1, z2, z3, params.b0, params.wo, dt);

    % 控制律
    e1 = phi_ref - z1;
    e2 = 0 - z2;  % 期望角速率为 0
    u0 = params.k1 * e1 + params.k2 * e2;
    u = (u0 - z3) / params.b0;

    % 限幅
    u = max(min(u, u_max), -u_max);

    u_prev = u;
end
```

### 6.2 级联 ADRC 结构

```
┌─────────────────────────────────────────────────┐
│           四旋翼级联 ADRC 控制                    │
├─────────────────────────────────────────────────┤
│                                                 │
│  位置参考 ──→ [位置 ADRC] ──→ 姿态参考           │
│                    │                            │
│                    ↓                            │
│              [姿态 ADRC] ──→ 力矩               │
│                    │                            │
│                    ↓                            │
│              [角速率 ADRC] ──→ 电机控制量         │
│                                                 │
│  每个环路独立设计 ADRC                            │
│  ESO 分别估计各环路的总扰动                      │
│                                                 │
└─────────────────────────────────────────────────┘
```

### 6.3 ADRC 扰动估计能力

```matlab
% 扰动估计可视化
figure;
subplot(2,1,1);
plot(t, d_actual, 'b', t, z3_estimated, 'r--');
xlabel('时间 (s)');
ylabel('扰动');
legend('实际扰动', 'ESO 估计');
title('ADRC 扰动估计');

subplot(2,1,2);
plot(t, d_actual - z3_estimated);
xlabel('时间 (s)');
ylabel('估计误差');
title('扰动估计误差');
```

---

## 7. ADRC 的改进与发展

### 7.1 改进方向

| 改进方向 | 方法 | 优势 |
|---------|------|------|
| 非线性 ESO | 使用 fal 函数 | 更好的估计性能 |
| 自适应 ADRC | 在线调整 b0 | 适应参数变化 |
| 模型辅助 ADRC | 利用部分模型信息 | 提高估计精度 |
| 有限时间 ESO | 有限时间收敛 | 更快的估计速度 |
| 离散 ADRC | 针对离散系统设计 | 更适合数字实现 |

### 7.2 ADRC 与其他方法结合

```
ADRC 与其他控制方法结合：

ADRC + MPC
├── ADRC 估计扰动
├── MPC 处理约束
└── 互补优势

ADRC + 滑模
├── ADRC 估计扰动
├── 滑模保证鲁棒性
└── 减小抖振

ADRC + 模糊
├── 模糊调整 ADRC 参数
├── 适应不同工作点
└── 提高自适应能力
```

---

## 8. 总结

### 8.1 ADRC 的关键特性

| 特性 | 说明 |
|------|------|
| 不依赖模型 | 只需知道控制增益 b 的大致范围 |
| 主动抗扰 | ESO 实时估计并补偿总扰动 |
| 结构简单 | 容易理解和实现 |
| 参数直观 | 带宽法整定，物理意义明确 |
| 适用广泛 | 适用于多种系统类型 |

### 8.2 选择建议

```
何时选择 ADRC？
├── 模型不确定性大
├── 外部扰动强
├── 需要快速扰动抑制
├── 不想花时间建模
└── PID 性能不满足要求

何时选择 PID？
├── 模型较精确
├── 扰动较小
├── 计算资源有限
├── 工程经验丰富
└── 需要快速部署
```

---

## 思考题

1. ADRC 的"总扰动"概念与 PID 的积分项有什么本质区别？
2. ESO 带宽 $\omega_o$ 的选择需要考虑哪些因素？过大和过小会有什么问题？
3. 为什么 ADRC 特别适合无人机控制？请从无人机的特点分析。
4. 如何在线估计控制增益 $b$？这对 ADRC 性能有什么影响？
5. ADRC 与 $H_\infty$ 控制在扰动处理方面有什么异同？

<details>
<summary>参考答案</summary>

1. 本质区别：(1) PID 积分项被动累积历史误差，响应慢；(2) ADRC 的 ESO 主动估计当前总扰动，可实时补偿；(3) 积分项无法区分扰动类型，ESO 可估计扰动的具体形式；(4) 积分项可能导致饱和，ESO 不会。

2. ESO 带宽选择：(1) $\omega_o$ 过大：估计快速但放大噪声，可能导致控制量抖动；(2) $\omega_o$ 过小：估计滞后，扰动抑制能力下降；(3) 经验：$\omega_o \approx (3 \sim 10) \omega_c$；(4) 需要考虑传感器噪声水平和采样率。

3. ADRC 适合无人机的原因：(1) 无人机模型难以精确建立（气动参数不确定）；(2) 外部扰动强（风扰动、地面效应）；(3) 需要快速扰动抑制（姿态稳定）；(4) ADRC 不依赖精确模型，适合快速部署；(5) 级联结构与 ADRC 天然契合。

4. 在线估计 b 的方法：(1) 使用 ESO 估计：将 b 也作为待估计参数；(2) 使用自适应律：$\hat{b} = \hat{b} + \gamma \cdot e \cdot u$；(3) 影响：b 估计不准会导致控制量偏大或偏小，影响性能；(4) 通常 b 的估计误差在 50% 以内时系统仍可稳定。

5. ADRC vs H_infinity：(1) 相同点：都处理不确定性和扰动；(2) 不同点：ADRC 通过估计补偿扰动，H_infinity 通过鲁棒设计保证性能；(3) ADRC 不需要精确模型，H_infinity 需要不确定性界；(4) ADRC 是时域方法，H_infinity 是频域方法；(5) 工程中可结合使用。

</details>
