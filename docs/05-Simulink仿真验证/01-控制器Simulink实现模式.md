# 05-Simulink仿真验证：控制器Simulink实现模式

> 预计阅读：30 分钟 | 前置知识：Simulink基础、MATLAB编程、控制理论

---

## 1. Simulink 控制器实现概述

Simulink 是 MATLAB 的图形化仿真环境，特别适合控制系统的设计、仿真和验证。

### 1.1 常用实现方式

```
┌─────────────────────────────────────────────────┐
│              控制器 Simulink 实现方式              │
├─────────────────────────────────────────────────┤
│                                                 │
│  1. 内置模块                                     │
│     ├── PID Controller                          │
│     ├── Transfer Fcn                            │
│     └── State-Space                             │
│                                                 │
│  2. MATLAB Function Block                       │
│     ├── 编写 MATLAB 函数                        │
│     ├── 灵活性高                                │
│     └── 支持代码生成                            │
│                                                 │
│  3. S-Function                                  │
│     ├── C/C++ 实现                              │
│     ├── 性能最优                                │
│     └── 适合嵌入式部署                          │
│                                                 │
│  4. Stateflow                                   │
│     ├── 状态机建模                              │
│     ├── 模式切换                                │
│     └── 逻辑控制                                │
│                                                 │
└─────────────────────────────────────────────────┘
```

### 1.2 选择建议

| 实现方式 | 适用场景 | 优点 | 缺点 |
|---------|---------|------|------|
| 内置模块 | 简单控制器 | 易用 | 灵活性低 |
| MATLAB Function | 复杂算法 | 灵活 | 性能中等 |
| S-Function | 高性能需求 | 最优性能 | 开发复杂 |
| Stateflow | 模式切换 | 直观 | 学习曲线 |

---

## 2. MATLAB Function Block

### 2.1 基本用法

```matlab
% MATLAB Function Block 示例
function u = PID_controller(e, e_prev, integral_prev, Kp, Ki, Kd, dt)
    % PID 控制器

    % 积分项
    integral = integral_prev + (e + e_prev) / 2 * dt;

    % 微分项
    derivative = (e - e_prev) / dt;

    % PID 输出
    u = Kp * e + Ki * integral + Kd * derivative;
end
```

### 2.2 ADRC 实现

```matlab
function [u, z1, z2, z3] = ADRC_block(r, y, u_prev, ...
    z1_prev, z2_prev, z3_prev, b0, wo, k1, k2, dt)
    % ADRC 控制器 MATLAB Function Block

    % ESO 更新
    e_obs = z1_prev - y;
    z1_dot = z2_prev - 2*wo*e_obs;
    z2_dot = z3_prev - wo^2*e_obs + b0*u_prev;
    z3_dot = -wo^3*e_obs;

    z1 = z1_prev + z1_dot * dt;
    z2 = z2_prev + z2_dot * dt;
    z3 = z3_prev + z3_dot * dt;

    % 控制律
    e1 = r - z1;
    e2 = -z2;
    u0 = k1 * e1 + k2 * e2;
    u = (u0 - z3) / b0;
end
```

### 2.3 滑模控制实现

```matlab
function u = SMC_block(x, x_d, x_dot_d, lambda, eta, phi)
    % 滑模控制 MATLAB Function Block

    % 误差
    e = x - x_d;
    edot = x_dot - x_dot_d;

    % 滑模面
    s = edot + lambda * e;

    % 等效控制
    u_eq = -lambda * edot;

    % 切换控制（饱和函数）
    if abs(s) > phi
        u_sw = -eta * sign(s);
    else
        u_sw = -eta * s / phi;
    end

    % 总控制
    u = u_eq + u_sw;
end
```

---

## 3. S-Function 实现

### 3.1 S-Function 基础

**S-Function 结构**：

```matlab
function [sys, x0, str, ts] = sfun_controller(t, x, u, flag, params)
    % S-Function 控制器

    switch flag
        case 0  % 初始化
            [sys, x0, str, ts] = mdlInitializeSizes(params);
        case 1  % 导数
            sys = mdlDerivatives(t, x, u, params);
        case 2  % 更新
            sys = mdlUpdate(t, x, u, params);
        case 3  % 输出
            sys = mdlOutputs(t, x, u, params);
        case 9  % 终止
            sys = [];
        otherwise
            error(['Unhandled flag = ', num2str(flag)]);
    end
end

function [sys, x0, str, ts] = mdlInitializeSizes(params)
    sizes = simsizes;
    sizes.NumContStates  = 0;
    sizes.NumDiscStates  = params.NumStates;
    sizes.NumOutputs     = params.NumOutputs;
    sizes.NumInputs      = params.NumInputs;
    sizes.DirFeedthrough = 1;
    sizes.NumSampleTimes = 1;
    sys = simsizes(sizes);
    x0 = zeros(params.NumStates, 1);
    str = [];
    ts = [params.Ts 0];
end

function sys = mdlOutputs(t, x, u, params)
    % 控制器输出
    % u: 输入（误差、状态等）
    % sys: 输出（控制量）

    Kp = params.Kp;
    Ki = params.Ki;
    Kd = params.Kd;

    e = u(1);
    integral = x(1);
    derivative = u(2);

    sys = Kp * e + Ki * integral + Kd * derivative;
end
```

### 3.2 C-MEX S-Function

```c
/* C-MEX S-Function 控制器 */
#define S_FUNCTION_NAME  sfun_pid_controller
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 3);  /* Kp, Ki, Kd */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 1);
    ssSetNumInputPorts(S, 1);
    ssSetInputPortWidth(S, 0, 1);
    ssSetNumOutputPorts(S, 1);
    ssSetOutputPortWidth(S, 0, 1);
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T *y = ssGetOutputPortRealSignal(S, 0);
    real_T *x = ssGetDiscStates(S);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S, 0);

    real_T Kp = mxGetPr(ssGetSFcnParam(S, 0))[0];
    real_T Ki = mxGetPr(ssGetSFcnParam(S, 1))[0];
    real_T Kd = mxGetPr(ssGetSFcnParam(S, 2))[0];

    real_T e = *uPtrs[0];
    real_T integral = x[0];

    y[0] = Kp * e + Ki * integral;
}
```

---

## 4. Stateflow 模式切换

### 4.1 状态机设计

```
┌─────────────────────────────────────────────────┐
│              飞行模式状态机                        │
├─────────────────────────────────────────────────┤
│                                                 │
│  ┌──────────┐                                   │
│  │  待机    │←──────────────────────┐           │
│  │  Idle    │                       │           │
│  └────┬─────┘                       │           │
│       │ 起飞指令                    │ 着陆完成   │
│       ↓                             │           │
│  ┌──────────┐                       │           │
│  │  起飞    │                       │           │
│  │  Takeoff │                       │           │
│  └────┬─────┘                       │           │
│       │ 到达目标高度                │           │
│       ↓                             │           │
│  ┌──────────┐                       │           │
│  │  悬停    │                       │           │
│  │  Hover   │                       │           │
│  └────┬─────┘                       │           │
│       │ 航点指令                    │           │
│       ↓                             │           │
│  ┌──────────┐                       │           │
│  │  航线    │───────────────────────┘           │
│  │  Mission │                                   │
│  └──────────┘                                   │
│                                                 │
└─────────────────────────────────────────────────┘
```

### 4.2 Stateflow 实现

```matlab
% Stateflow 状态机定义
% 状态：Idle, Takeoff, Hover, Mission, Land
% 事件：takeoff_cmd, reach_alt, waypoint_cmd, land_complete

chart {
    state Idle {
        on takeoff_cmd: transition(Takeoff);
    }

    state Takeoff {
        on reach_alt: transition(Hover);
    }

    state Hover {
        on waypoint_cmd: transition(Mission);
    }

    state Mission {
        on land_cmd: transition(Land);
    }

    state Land {
        on land_complete: transition(Idle);
    }
}
```

### 4.3 控制器切换

```matlab
function u = mode_switch_controller(mode, x, params)
    % 模式切换控制器

    switch mode
        case 'Idle'
            u = [0; 0; 0; 0];  % 零输出

        case 'Takeoff'
            u = takeoff_controller(x, params);

        case 'Hover'
            u = hover_controller(x, params);

        case 'Mission'
            u = mission_controller(x, params);

        case 'Land'
            u = land_controller(x, params);

        otherwise
            u = [0; 0; 0; 0];
    end
end
```

---

## 5. 子系统架构

### 5.1 模块化设计

```
┌─────────────────────────────────────────────────────────────┐
│                    四旋翼控制 Simulink 模型                   │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │  位置控制   │  │  姿态控制   │  │  电机混控   │        │
│  │  Subsystem  │→│  Subsystem  │→│  Subsystem  │        │
│  └─────────────┘  └─────────────┘  └─────────────┘        │
│         ↑              ↑              │                     │
│         │              │              ↓                     │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │  GPS/视觉   │  │    IMU     │  │   电调/电机  │        │
│  │  传感器     │  │   传感器   │  │   执行器    │        │
│  └─────────────┘  └─────────────┘  └─────────────┘        │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 5.2 子系统封装

```matlab
% 创建封装子系统
function create_masked_subsystem(model_name)
    % 创建 PID 控制器子系统
    subsys_path = [model_name '/PID_Controller'];

    % 添加掩码
    mask = Simulink.Mask.create(subsys_path);

    % 添加参数
    mask.addParameter('Kp', '5.0', 'Kp gain');
    mask.addParameter('Ki', '0.1', 'Ki gain');
    mask.addParameter('Kd', '0.05', 'Kd gain');

    % 添加图标
    mask.Display = 'disp(''PID'')';
end
```

---

## 6. 代码生成

### 6.1 嵌入式代码生成

```matlab
% 配置代码生成
% 1. 打开 Configuration Parameters
set_param(model_name, 'SolverType', 'Fixed-step');
set_param(model_name, 'Solver', 'ode4');
set_param(model_name, 'FixedStep', '0.001');

% 2. 选择目标硬件
set_param(model_name, 'TargetHWDeviceType', 'ARM Compatible->ARM Cortex-M');

% 3. 生成代码
slbuild(model_name);
```

### 6.2 代码优化

```matlab
% 代码优化设置
% 1. 数据类型优化
set_param(model_name, 'OptimizeBlockIOStorage', 'on');
set_param(model_name, 'ExpressionFolding', 'on');

% 2. 内存优化
set_param(model_name, 'BufferReuse', 'on');

% 3. 执行效率优化
set_param(model_name, 'InlineParams', 'on');
```

---

## 7. 最佳实践

### 7.1 模型组织

```
推荐的模型组织结构：

model_name.slx
├── Reference_Model（被控对象）
├── Controller（控制器）
│   ├── Position_Controller
│   ├── Attitude_Controller
│   └── Rate_Controller
├── Sensors（传感器模型）
├── Environment（环境模型）
└── Visualization（可视化）
```

### 7.2 调试技巧

| 技巧 | 说明 |
|------|------|
| 信号标注 | 给重要信号添加标签 |
| 断点调试 | 在 MATLAB Function 中设置断点 |
| 数据记录 | 使用 To Workspace 模块 |
| 性能分析 | 使用 Simulink Profiler |

---

## 思考题

1. MATLAB Function Block 和 S-Function 各有什么优缺点？
2. Stateflow 在飞控系统中主要用于什么场景？
3. 如何设计模块化的 Simulink 控制器模型？
4. 嵌入式代码生成需要注意哪些问题？
5. 如何验证 Simulink 生成的代码与模型一致性？

<details>
<summary>参考答案</summary>

1. 对比：(1) MATLAB Function：开发快，灵活性高，但性能中等；(2) S-Function：性能最优，支持 C 代码，但开发复杂；(3) 选择：原型验证用 MATLAB Function，嵌入式部署用 S-Function。

2. Stateflow 应用：(1) 飞行模式切换（起飞、悬停、航线、着陆）；(2) 故障处理逻辑；(3) 安全检查逻辑；(4) 任务状态管理。

3. 模块化设计：(1) 按功能划分子系统；(2) 使用封装隐藏内部细节；(3) 标准化接口（输入输出）；(4) 便于复用和测试。

4. 注意事项：(1) 选择合适的数据类型；(2) 避免动态内存分配；(3) 处理好离散化；(4) 验证代码与模型一致性。

5. 验证方法：(1) 软件在环仿真（SIL）；(2) 处理器在环仿真（PIL）；(3) 代码覆盖率分析；(4) 与原始模型对比测试。

</details>
