# pid-kalman-joint-control

MATLAB simulation code for a single-joint robotic actuator under PID control with
Kalman-filter-based multi-sensor fusion (gyroscope + encoder). This repository
contains all code required to reproduce the results, figures, and performance
metrics reported in the associated manuscript.

> 本仓库包含论文所用的全部 MATLAB 仿真代码：单关节机器人在 PID 控制下，
> 结合卡尔曼滤波对陀螺仪与编码器进行多传感器融合。运行代码即可复现论文中
> 所有图与性能指标。

## Data availability / 数据说明

This study uses **no empirical or third-party data**. All data are generated
entirely by numerical simulation within `main.m`. Running the script reproduces
every figure and reported metric.

本研究不使用任何实测数据或第三方数据集，所有数据均由 `main.m` 仿真生成。
运行脚本即可复现全部结果。

A fixed random seed (`rng(42)`) is set at the top of `main.m` so that the
stochastic sensor noise is reproducible and the reported numerical results can
be regenerated exactly.

代码开头设置了固定随机种子 `rng(42)`，保证含噪声的仿真结果可被精确复现。

## Requirements / 运行环境

- MATLAB (R2018b or later recommended)
- No additional toolboxes required (uses base MATLAB only)

## How to run / 运行方法

1. Clone or download this repository.
2. Open MATLAB and set the working directory to the repository folder.
3. Run:

   ```matlab
   main
   ```

4. The script will:
   - Run the closed-loop simulation (0–5 s, step size 0.001 s).
   - Generate 10 figures covering tracking performance, PID error, control
     voltage, current/torque, raw sensor measurements, fusion results, error
     comparison, Kalman gain, estimation covariance, and angular-velocity tracking.
   - Print key performance metrics to the Command Window (steady-state error,
     gyroscope MSE, encoder MSE, and fused MSE).

## File overview / 文件说明

- `main.m` — complete simulation script (model setup, PID control loop,
  dynamics, sensor noise, Kalman filter, plotting, and metric output).

## Citation / 引用

If you use this code, please cite the associated manuscript.
(本代码如被使用，请引用对应论文。)
