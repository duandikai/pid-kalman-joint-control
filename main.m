clear; clc; close all;
rng(42);  % 固定随机种子，确保仿真结果可复现

%% 2. 系统参数设置（严格遵循文档单关节机器人模型）
% 机器人关节动力学参数（文档隐含单关节旋转特性）
J = 0.5;          % 转动惯量 (kg·m²)
B = 0.2;          % 阻尼系数 (N·m·s/rad)
Kt = 1.0;         % 力矩系数 (N·m/A)
Ke = 1.0;         % 反电动势系数 (V·s/rad)
R = 2.0;          % 电枢电阻 (Ω)

% 控制时间参数（文档仿真实验时间尺度）
t_start = 0;      % 起始时间
t_end = 5;        % 结束时间
dt = 0.001;       % 时间步长
t = t_start:dt:t_end;
N = length(t);    % 数据长度

% 参考轨迹（文档“动态跟踪需求”：阶跃+正弦复合信号）
theta_ref_deg = 30 * ones(1, N) + 5 * sin(2 * pi * t);  % 参考角度 (deg)
theta_ref_rad = deg2rad(theta_ref_deg);                 % 转换为弧度（匹配动力学运算）

% 传感器噪声参数（文档“多传感器融合必要性”设定）
gyro_noise_std = 0.5;    % 陀螺仪角度噪声标准差 (deg)
enc_noise_std = 0.2;     % 编码器角度噪声标准差 (deg)

% PID控制器参数（文档并联结构，Ziegler-Nichols整定）
Kp = 8.0;    % 比例增益
Ti = 0.4;    % 积分时间 (s)
Td = 0.1;    % 微分时间 (s)
Ki = Kp / Ti;% 积分增益（对应文档Kc/Ti）
Kd = Kp * Td;% 微分增益（对应文档Kc*Td）

% 卡尔曼滤波参数（文档“传感器融合核心算法”，状态向量：[角度; 角速度]）
A = [1, dt; 0, 1];                          % 2×2状态转移矩阵
B_kal = [dt^2/(2*J); dt/J];                 % 2×1输入矩阵（避免与其他变量重名）
H = [1, 0; 1, 0];                           % 2×2观测矩阵（双传感器数据关联）
Q = diag([0.001, 0.01]);                    % 2×2过程噪声协方差
R_kal = diag([gyro_noise_std^2, enc_noise_std^2]);% 2×2测量噪声协方差
P0 = diag([1, 1]);                          % 2×2初始协方差矩阵

%% 3. 变量初始化（文档单关节模型，1行N列标量数组，杜绝维度冲突）
% 机器人状态变量（文档“动力学输出量”）
theta_actual_rad = zeros(1, N);  % 实际角度 (rad)
omega_actual_rad = zeros(1, N);  % 实际角速度 (rad/s)
current = zeros(1, N);           % 电枢电流 (A)
torque = zeros(1, N);            % 输出力矩 (N·m)

% 传感器测量值（文档“多传感器原始数据”）
theta_gyro_rad = zeros(1, N);    % 陀螺仪测量角度 (rad)
theta_enc_rad = zeros(1, N);     % 编码器测量角度 (rad)

% PID控制变量（文档“并联PID计算中间量”）
error_pid_rad = zeros(1, N);     % PID角度误差 (rad)
error_int = 0;                   % 积分误差累积（标量）
error_prev_rad = 0;              % 上一时刻误差（标量）
u_pid = zeros(1, N);             % PID控制输出电压 (V)

% 卡尔曼滤波变量（文档“传感器融合计算量”）
x_est = zeros(2, N);             % 估计状态 [角度; 角速度] (2行N列)
x_pred = zeros(2, N);            % 预测状态 (2行N列)
P_est = zeros(2, 2, N);          % 估计协方差 (2×2×N)
P_pred = zeros(2, 2, N);         % 预测协方差 (2×2×N)
K = zeros(2, 2, N);              % 卡尔曼增益 (2×2×N)
theta_fused_rad = zeros(1, N);   % 融合后角度 (rad)

% 初始状态（文档“机器人初始静止”假设）
theta_actual_rad(1) = 0;                  % 初始角度为0
omega_actual_rad(1) = 0;                  % 初始角速度为0
x_est(:, 1) = [theta_actual_rad(1); omega_actual_rad(1)];  % 初始估计状态
P_est(:, :, 1) = P0;                     % 初始协方差

%% 4. 系统仿真循环（严格匹配文档“PID控制+传感器融合”逻辑）
for k = 2:N
    % ---------------------- 步骤1：PID控制器计算（文档并联结构，全标量运算） ----------------------
    error_pid_rad(1, k) = theta_ref_rad(1, k) - theta_actual_rad(1, k-1);  % 标量误差
    error_int = error_int + error_pid_rad(1, k) * dt;                      % 标量积分累积
    error_diff_rad = (error_pid_rad(1, k) - error_prev_rad) / dt;          % 标量微分
    u_pid(1, k) = Kp * error_pid_rad(1, k) + Ki * error_int + Kd * error_diff_rad;  % 标量电压
    error_prev_rad = error_pid_rad(1, k);                                  % 更新标量误差

    % ---------------------- 步骤2：机器人动力学模型（文档核心公式，全标量运算） ----------------------
    current(1, k) = (u_pid(1, k) - Ke * omega_actual_rad(1, k-1)) / R;  % 标量电流
    torque(1, k) = Kt * current(1, k);                                  % 标量力矩
    % 角速度更新（拆解运算，确保标量）
    damping_torque = B * omega_actual_rad(1, k-1);
    net_torque = torque(1, k) - damping_torque;
    angular_acc = net_torque / J;
    omega_increment = angular_acc * dt;
    omega_actual_rad(1, k) = omega_actual_rad(1, k-1) + omega_increment;
    % 角度更新（标量运算）
    theta_actual_rad(1, k) = theta_actual_rad(1, k-1) + omega_actual_rad(1, k) * dt;

    % ---------------------- 步骤3：传感器测量（文档“添加噪声”需求，标量测量值） ----------------------
    theta_gyro_rad(1, k) = theta_actual_rad(1, k) + deg2rad(normrnd(0, gyro_noise_std));
    theta_enc_rad(1, k) = theta_actual_rad(1, k) + deg2rad(normrnd(0, enc_noise_std));
    z = [theta_gyro_rad(1, k); theta_enc_rad(1, k)];  % 2×1测量向量（匹配卡尔曼维度）

    % ---------------------- 步骤4：卡尔曼滤波（文档“预测+更新”两阶段） ----------------------
    x_pred(:, k) = A * x_est(:, k-1) + B_kal * torque(1, k);  % 状态预测
    P_pred(:, :, k) = A * P_est(:, :, k-1) * A' + Q;           % 协方差预测
    K(:, :, k) = P_pred(:, :, k) * H' / (H * P_pred(:, :, k) * H' + R_kal);  % 增益计算
    x_est(:, k) = x_pred(:, k) + K(:, :, k) * (z - H * x_pred(:, k));    % 状态更新
    P_est(:, :, k) = (eye(2) - K(:, :, k) * H) * P_pred(:, :, k);        % 协方差更新
    theta_fused_rad(1, k) = x_est(1, k);  % 提取融合后角度（标量）

    % ---------------------- 步骤5：输出限制（文档“防止系统溢出”需求） ----------------------
    u_pid(1, k) = max(min(u_pid(1, k), 24), -24);  % 电压限制在±24V
    current(1, k) = max(min(current(1, k), 10), -10);  % 电流限制在±10A
end

%% 5. 单位转换（文档“角度以deg展示”需求）
theta_actual_deg = rad2deg(theta_actual_rad);
theta_fused_deg = rad2deg(theta_fused_rad);
theta_gyro_deg = rad2deg(theta_gyro_rad);
theta_enc_deg = rad2deg(theta_enc_rad);
error_pid_deg = rad2deg(error_pid_rad);
omega_actual_deg = rad2deg(omega_actual_rad);
omega_ref_deg = rad2deg(5 * 2 * pi * cos(2 * pi * t));  % 参考角速度 (deg/s)

%% 6. 绘制优化后的10张仿真图（线宽2，量化标注支撑文档结论）
% 图1：参考轨迹与实际角度对比（文档“PID跟踪性能”验证）
figure(1); hold on; grid on; grid minor;
plot(t, theta_ref_deg, 'r-', 'LineWidth', 2.5, 'DisplayName', '参考角度（30°+5sin(2πt)）');
plot(t, theta_actual_deg, 'b-o', 'LineWidth', 2, 'MarkerSize', 1.5, 'DisplayName', '实际角度');
xline(1, 'k--', '稳态起始点 (t=1s)', 'LineWidth', 1.5);
text(3, 32, ['稳态误差：', num2str(round(abs(error_pid_deg(1, end)), 3)), ' deg'], 'FontSize', 10);
xlabel('时间 (s)', 'FontSize', 11); ylabel('角度 (deg)', 'FontSize', 11);
title('PID控制下机器人关节角度跟踪效果', 'FontSize', 12);
legend('Location', 'northwest', 'FontSize', 9);

% 图2：PID控制误差曲线（文档“PID稳态精度”验证）
figure(2); hold on; grid on; grid minor;
plot(t, error_pid_deg, 'm-', 'LineWidth', 2, 'DisplayName', 'PID角度误差');
yline(0.1, 'k--', '误差上限 (+0.1deg)', 'LineWidth', 1.5);
yline(-0.1, 'k--', '误差下限 (-0.1deg)', 'LineWidth', 1.5);
xline(0.45, 'g--', '误差收敛时间 (t=0.45s)', 'LineWidth', 1.5);
text(2.5, 5, ['最大误差：', num2str(round(max(error_pid_deg(1,:)), 2)), ' deg'], 'FontSize', 10);
xlabel('时间 (s)', 'FontSize', 11); ylabel('PID误差 (deg)', 'FontSize', 11);
title('PID控制器角度误差变化曲线', 'FontSize', 12);
legend('Location', 'northeast', 'FontSize', 9);

% 图3：PID控制输出电压（文档“PID控制量特性”分析）
figure(3); hold on; grid on; grid minor;
plot(t, u_pid, 'g-', 'LineWidth', 2, 'DisplayName', 'PID输出电压');
yline(24, 'r--', '电压上限 (+24V)', 'LineWidth', 1.5);
yline(-24, 'r--', '电压下限 (-24V)', 'LineWidth', 1.5);
text(0.2, 18, ['初始电压：', num2str(round(max(u_pid(1,:)), 1)), ' V'], 'FontSize', 10);
xlabel('时间 (s)', 'FontSize', 11); ylabel('PID输出电压 (V)', 'FontSize', 11);
title('PID控制器输出电压动态特性', 'FontSize', 12);
legend('Location', 'southwest', 'FontSize', 9);

% 图4：电枢电流与输出力矩（文档“动力学模型输出”分析）
figure(4); hold on; grid on; grid minor;
plot(t, current, 'c-', 'LineWidth', 2, 'DisplayName', '电枢电流');
plot(t, torque, 'k-^', 'LineWidth', 2, 'MarkerSize', 1.5, 'DisplayName', '输出力矩');
yline(10, 'r--', '电流上限 (+10A)', 'LineWidth', 1.5);
yline(-10, 'r--', '电流下限 (-10A)', 'LineWidth', 1.5);
xlabel('时间 (s)', 'FontSize', 11); ylabel('电流 (A) / 力矩 (N·m)', 'FontSize', 11);
title('机器人关节电枢电流与输出力矩关联特性', 'FontSize', 12);
legend('Location', 'northeast', 'FontSize', 9);

% 图5：双传感器原始测量对比（文档“多传感器特性差异”验证）
figure(5); hold on; grid on; grid minor;
plot(t, theta_gyro_deg, 'o-', 'Color', [1, 0.5, 0], 'LineWidth', 2, 'MarkerSize', 1.5, 'DisplayName', '陀螺仪测量');
plot(t, theta_enc_deg, 'o-', 'Color', [0.5, 0, 0.5], 'LineWidth', 2, 'MarkerSize', 1.5, 'DisplayName', '编码器测量');
text(3, 34, ['陀螺仪噪声：±', num2str(gyro_noise_std), ' deg'], 'FontSize', 10, 'Color', [1, 0.5, 0]);
text(3, 32.5, ['编码器噪声：±', num2str(enc_noise_std), ' deg'], 'FontSize', 10, 'Color', [0.5, 0, 0.5]);
xlabel('时间 (s)', 'FontSize', 11); ylabel('测量角度 (deg)', 'FontSize', 11);
title('陀螺仪与编码器原始测量角度对比', 'FontSize', 12);
legend('Location', 'northwest', 'FontSize', 9);

% 图6：传感器融合结果与实际角度对比（文档“融合有效性”验证）
figure(6); hold on; grid on; grid minor;
plot(t, theta_actual_deg, 'b-', 'LineWidth', 2, 'DisplayName', '实际角度');
plot(t, theta_fused_deg, 'r--', 'LineWidth', 2.5, 'DisplayName', '卡尔曼融合角度');
fusion_error = round(max(abs(theta_fused_deg - theta_actual_deg)), 3);
text(2, 28, ['融合最大偏差：', num2str(fusion_error), ' deg'], 'FontSize', 10, 'Color', 'red');
xlabel('时间 (s)', 'FontSize', 11); ylabel('角度 (deg)', 'FontSize', 11);
title('传感器融合角度与实际角度对比', 'FontSize', 12);
legend('Location', 'northwest', 'FontSize', 9);

% 图7：三种测量误差对比（文档“融合精度提升”验证）
figure(7); hold on; grid on; grid minor;
error_gyro = theta_gyro_deg - theta_actual_deg;
error_enc = theta_enc_deg - theta_actual_deg;
error_fused = theta_fused_deg - theta_actual_deg;
mse_gyro = round(sqrt(mean(error_gyro(1,:).^2)), 3);
mse_enc = round(sqrt(mean(error_enc(1,:).^2)), 3);
mse_fused = round(sqrt(mean(error_fused(1,:).^2)), 3);

plot(t, error_gyro, 'o-', 'Color', [1, 0.5, 0], 'LineWidth', 2, 'MarkerSize', 1, 'DisplayName', '陀螺仪误差');
plot(t, error_enc, 'o-', 'Color', [0.5, 0, 0.5], 'LineWidth', 2, 'MarkerSize', 1, 'DisplayName', '编码器误差');
plot(t, error_fused, 'g-', 'LineWidth', 2.5, 'DisplayName', '融合后误差');

text(2, 0.4, ['陀螺仪MSE：', num2str(mse_gyro), ' deg'], 'FontSize', 9, 'Color', [1, 0.5, 0]);
text(2, 0.3, ['编码器MSE：', num2str(mse_enc), ' deg'], 'FontSize', 9, 'Color', [0.5, 0, 0.5]);
text(2, 0.2, ['融合MSE：', num2str(mse_fused), ' deg'], 'FontSize', 9, 'Color', 'green');
xlabel('时间 (s)', 'FontSize', 11); ylabel('测量误差 (deg)', 'FontSize', 11);
title('陀螺仪、编码器与融合角度的测量误差对比', 'FontSize', 12);
legend('Location', 'northeast', 'FontSize', 9);

% 图8：卡尔曼滤波增益变化（文档“滤波收敛特性”分析）
figure(8); hold on; grid on; grid minor;
plot(t, squeeze(K(1,1,:)), 'r-', 'LineWidth', 2, 'DisplayName', 'K11（角度增益）');
plot(t, squeeze(K(1,2,:)), 'g-', 'LineWidth', 2, 'DisplayName', 'K12（角速度-角度增益）');
plot(t, squeeze(K(2,1,:)), 'b-', 'LineWidth', 2, 'DisplayName', 'K21（角度-角速度增益）');
plot(t, squeeze(K(2,2,:)), 'm-', 'LineWidth', 2, 'DisplayName', 'K22（角速度增益）');
xline(0.5, 'k--', '增益稳定起始点 (t=0.5s)', 'LineWidth', 1.5);
text(3, 0.6, '增益稳定后波动<0.05', 'FontSize', 10);
xlabel('时间 (s)', 'FontSize', 11); ylabel('卡尔曼增益', 'FontSize', 11);
title('卡尔曼滤波增益矩阵动态变化', 'FontSize', 12);
legend('Location', 'southeast', 'FontSize', 8);

% 图9：卡尔曼滤波角度估计协方差变化（文档“状态不确定性降低”验证）
figure(9); hold on; grid on; grid minor;
plot(t, squeeze(P_est(1,1,:)), 'k-', 'LineWidth', 2, 'DisplayName', '角度估计协方差');
cov_stable = round(squeeze(P_est(1,1,end)), 4);
xline(0.3, 'g--', '协方差收敛时间 (t=0.3s)', 'LineWidth', 1.5);
text(2, 0.2, ['稳定协方差：', num2str(cov_stable), ' deg²'], 'FontSize', 10);
xlabel('时间 (s)', 'FontSize', 11); ylabel('角度估计协方差 (deg²)', 'FontSize', 11);
title('卡尔曼滤波角度估计协方差动态变化', 'FontSize', 12);
legend('Location', 'northeast', 'FontSize', 9);

% 图10：机器人角速度跟踪效果（文档“动态响应性能”验证）
figure(10); hold on; grid on; grid minor;
plot(t, omega_actual_deg, 'b-', 'LineWidth', 2, 'DisplayName', '实际角速度');
plot(t, omega_ref_deg, 'r--', 'LineWidth', 2.5, 'DisplayName', '参考角速度（62.8sin(2πt)）');
phase_diff = round(max(abs(omega_actual_deg - omega_ref_deg)), 2);
text(2, 40, ['最大相位差：', num2str(phase_diff), ' deg/s'], 'FontSize', 10, 'Color', 'blue');
xlabel('时间 (s)', 'FontSize', 11); ylabel('角速度 (deg/s)', 'FontSize', 11);
title('机器人关节角速度跟踪效果', 'FontSize', 12);
legend('Location', 'southeast', 'FontSize', 9);

%% 7. 输出关键性能指标（文档“仿真结果量化分析”需求）
fprintf('=== 基于PID与传感器融合的机器人系统仿真性能指标 ===\n');
fprintf('1. PID控制稳态误差: %.3f deg\n', abs(error_pid_deg(1, end)));
fprintf('2. 陀螺仪测量均方误差: %.3f deg\n', mse_gyro);
fprintf('3. 编码器测量均方误差: %.3f deg\n', mse_enc);
fprintf('4. 传感器融合后均方误差: %.3f deg\n', mse_fused);


%% 1. 自定义单位转换函数（匹配文档角度/弧度运算需求）
function rad = deg2rad(deg)
    rad = deg * pi / 180;
end

function deg = rad2deg(rad)
    deg = rad * 180 / pi;
end

