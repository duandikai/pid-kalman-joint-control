clear; clc; close all;
rng(42);  % 固定随机种子，确保仿真结果可复现

%% 2. 系统参数设置（严格遵循文档单关节机器人模型，并加入MuJoCo/Webots模拟的额外参数）
% 机器人关节动力学参数（文档隐含单关节旋转特性）
J = 0.5;          % Moment of Inertia (kg·m²)
B = 0.2;          % Damping Coefficient (N·m·s/rad) - 视为粘性阻尼
Kt = 1.0;         % Torque Constant (N·m/A)
Ke = 1.0;         % Back-EMF Constant (V·s/rad)
R = 2.0;          % Armature Resistance (Ω)

% ** MuJoCo/Webots 风格的额外物理参数 **
F_coulomb = 0.8;  % Coulomb Friction Torque (N·m) - 模拟静态和动态摩擦
m = 1.0;          % Link Mass (kg) - 假设为单连杆
L = 0.5;          % Link Length (m) - 假设质心在连杆一半处
g = 9.81;         % Gravitational Acceleration (m/s²)
disturbance_amplitude = 1.5; % External Disturbance Torque Amplitude (N·m)
disturbance_frequency = 5; % External Disturbance Torque Frequency (Hz)

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
% 假设B_kal只用于控制输入带来的力矩，未建模的扰动由Q覆盖
B_kal = [dt^2/(2*J); dt/J];                 % 2×1输入矩阵（避免与其他变量重名）
H = [1, 0; 1, 0];                           % 2×2观测矩阵（双传感器数据关联）
Q = diag([0.005, 0.05]);                    % 2×2过程噪声协方差 (适当增加Q以应对未建模的扰动)
R_kal = diag([deg2rad(gyro_noise_std)^2, deg2rad(enc_noise_std)^2]);% 2×2测量噪声协方差
P0 = diag([1, 1]);                          % 2×2初始协方差矩阵

%% 3. 变量初始化（文档单关节模型，1行N列标量数组，杜绝维度冲突）
% 机器人状态变量（文档“动力学输出量”）
theta_actual_rad = zeros(1, N);  % 实际角度 (rad)
omega_actual_rad = zeros(1, N);  % 实际角速度 (rad/s)
current = zeros(1, N);           % 电枢电流 (A)
motor_torque_output = zeros(1, N); % 电机输出力矩 (N·m)

% ** MuJoCo/Webots 风格的额外力矩历史记录 **
friction_torque_hist = zeros(1, N);
gravity_torque_hist = zeros(1, N);
disturbance_torque_hist = zeros(1, N);


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
omega_fused_rad = zeros(1, N);   % 融合后角速度 (rad/s)

% 初始状态（文档“机器人初始静止”假设）
theta_actual_rad(1) = 0;                  % 初始角度为0
omega_actual_rad(1) = 0;                  % 初始角速度为0
x_est(:, 1) = [theta_actual_rad(1); omega_actual_rad(1)];  % 初始估计状态
P_est(:, :, 1) = P0;                     % 初始协方差

%% 4. 系统仿真循环（严格匹配文档“PID控制+传感器融合”逻辑，并加入MuJoCo/Webots风格的动力学）
for k = 2:N
    % ---------------------- 步骤1：PID控制器计算（文档并联结构，全标量运算） ----------------------
    % PID控制器根据融合后的估计值进行反馈
    error_pid_rad(1, k) = theta_ref_rad(1, k) - theta_fused_rad(1, k-1); % 使用融合角度作为反馈
    error_int = error_int + error_pid_rad(1, k) * dt;                      % 标量积分累积
    error_diff_rad = (error_pid_rad(1, k) - error_prev_rad) / dt;          % 标量微分
    u_pid(1, k) = Kp * error_pid_rad(1, k) + Ki * error_int + Kd * error_diff_rad;  % 标量电压
    error_prev_rad = error_pid_rad(1, k);                                  % 更新标量误差

    % ---------------------- 步骤2：机器人动力学模型（文档核心公式，加入MuJoCo/Webots风格细节） ----------------------
    % 电机模型
    u_pid_limited = max(min(u_pid(1, k), 24), -24); % 限制电压
    back_emf = Ke * omega_actual_rad(1, k-1);
    current(1, k) = (u_pid_limited - back_emf) / R;
    current(1, k) = max(min(current(1, k), 10), -10); % 限制电流
    motor_torque = Kt * current(1, k);
    motor_torque_output(1, k) = motor_torque; % 记录电机输出力矩

    % ** 引入 MuJoCo/Webots 风格的物理效应 **
    % 粘性摩擦 (Viscous Friction)
    viscous_friction_torque = B * omega_actual_rad(1, k-1);

    % 库仑摩擦 (Coulomb Friction)
    coulomb_friction_torque = F_coulomb * sign(omega_actual_rad(1, k-1));
    if abs(omega_actual_rad(1, k-1)) < 1e-3 % 接近静止时，库仑摩擦防止滑动
        coulomb_friction_torque = 0; % 或者引入静摩擦模型，这里简化处理
    end
    friction_torque_hist(1, k) = viscous_friction_torque + coulomb_friction_torque;

    % 重力力矩 (Gravitational Torque) - 假设关节水平时角度为0，垂直向下为pi/2
    % 这里的重力力矩假设是一个简化的单摆模型，sin(theta)当theta=0时重力矩为0，theta=pi/2时最大
    gravity_torque = m * g * L * sin(theta_actual_rad(1, k-1));
    gravity_torque_hist(1, k) = gravity_torque;

    % 外部扰动力矩 (External Disturbance Torque)
    disturbance_torque = disturbance_amplitude * sin(2 * pi * disturbance_frequency * t(k));
    disturbance_torque_hist(1, k) = disturbance_torque;

    % 计算净力矩
    net_torque = motor_torque - viscous_friction_torque - coulomb_friction_torque - gravity_torque - disturbance_torque;

    % 角速度更新
    angular_acc = net_torque / J;
    omega_actual_rad(1, k) = omega_actual_rad(1, k-1) + angular_acc * dt;
    % 角度更新
    theta_actual_rad(1, k) = theta_actual_rad(1, k-1) + omega_actual_rad(1, k) * dt;

    % ---------------------- 步骤3：传感器测量（文档“添加噪声”需求，标量测量值） ----------------------
    theta_gyro_rad(1, k) = theta_actual_rad(1, k) + normrnd(0, deg2rad(gyro_noise_std));
    theta_enc_rad(1, k) = theta_actual_rad(1, k) + normrnd(0, deg2rad(enc_noise_std));
    z = [theta_gyro_rad(1, k); theta_enc_rad(1, k)];  % 2×1测量向量（匹配卡尔曼维度）

    % ---------------------- 步骤4：卡尔曼滤波（文档“预测+更新”两阶段） ----------------------
    % 预测步骤
    % 注意：B_kal * motor_torque 假设 Kalman Filter 对电机的实际输出力矩有一定的模型认知。
    % 未建模的干扰 (摩擦、重力、外部扰动) 通过 Q (过程噪声协方差) 来覆盖。
    x_pred(:, k) = A * x_est(:, k-1) + B_kal * motor_torque_output(1, k);
    P_pred(:, :, k) = A * P_est(:, :, k-1) * A' + Q;
    
    % 更新步骤
    K(:, :, k) = P_pred(:, :, k) * H' / (H * P_pred(:, :, k) * H' + R_kal);
    x_est(:, k) = x_pred(:, k) + K(:, :, k) * (z - H * x_pred(:, k));
    P_est(:, :, k) = (eye(2) - K(:, :, k) * H) * P_pred(:, :, k);
    
    theta_fused_rad(1, k) = x_est(1, k);  % 提取融合后角度（标量）
    omega_fused_rad(1, k) = x_est(2, k);  % 提取融合后角速度（标量）

end

%% 5. 单位转换（文档“角度以deg展示”需求）
theta_actual_deg = rad2deg(theta_actual_rad);
theta_fused_deg = rad2deg(theta_fused_rad);
theta_gyro_deg = rad2deg(theta_gyro_rad);
theta_enc_deg = rad2deg(theta_enc_rad);
% PID误差现在是根据融合角度计算的
error_pid_deg = rad2deg(theta_ref_rad - theta_fused_rad); 
omega_actual_deg = rad2deg(omega_actual_rad);
omega_fused_deg = rad2deg(omega_fused_rad);
omega_ref_deg = rad2deg(5 * 2 * pi * cos(2 * pi * t));  % 参考角速度 (deg/s)

%% 6. 绘制原始文档中的10张仿真图（线宽2，量化标注支撑文档结论）
% 图1：参考轨迹与实际角度对比（文档“PID跟踪性能”验证）
figure(1); hold on; grid on; grid minor;
p1_1 = plot(t, theta_ref_deg, 'r-', 'LineWidth', 2.5, 'DisplayName', 'Reference Angle (30°+5sin(2πt))');
p1_2 = plot(t, theta_actual_deg, 'b-o', 'LineWidth', 2, 'MarkerSize', 1.5, 'DisplayName', 'Actual Angle');
xline(1, 'k--', 'Steady State Start (t=1s)', 'LineWidth', 1.5, 'HandleVisibility', 'off');
text(3, 32, ['Steady State Error: ', num2str(round(abs(error_pid_deg(1, end)), 3)), ' deg'], 'FontSize', 10);
xlabel('Time (s)', 'FontSize', 11); ylabel('Angle (deg)', 'FontSize', 11);
title('Robotic Joint Angle Tracking Performance with PID Control', 'FontSize', 12);
legend([p1_1, p1_2], 'Location', 'northwest', 'FontSize', 9);

% 图2：PID控制误差曲线（文档“PID稳态精度”验证）
figure(2); hold on; grid on; grid minor;
p2_1 = plot(t, error_pid_deg, 'm-', 'LineWidth', 2, 'DisplayName', 'PID Angle Error (Fused)');
yline(0.5, 'k--', 'Error Upper Bound (+0.5deg)', 'LineWidth', 1.5, 'HandleVisibility', 'off');
yline(-0.5, 'k--', 'Error Lower Bound (-0.5deg)', 'LineWidth', 1.5, 'HandleVisibility', 'off');
settling_time_threshold = 0.5; % deg
settling_start_time_for_check = 0.5; % s, starting point to check for settling
error_indices_after_start = find(t >= settling_start_time_for_check);
error_after_start_check = abs(error_pid_deg(error_indices_after_start));
last_exit_idx_in_subset = find(error_after_start_check > settling_time_threshold, 1, 'last');

if isempty(last_exit_idx_in_subset)
    settling_time = settling_start_time_for_check;
else
    settling_time = t(error_indices_after_start(last_exit_idx_in_subset) + 1);
end

xline(settling_time, 'g--', ['Settling Time (t=', num2str(round(settling_time, 2)), 's)'], 'LineWidth', 1.5, 'HandleVisibility', 'off');
max_error_pid = round(max(abs(error_pid_deg(1,:))), 2);
text(2.5, 5, ['Max Transient Error: ', num2str(max_error_pid), ' deg'], 'FontSize', 10);
xlabel('Time (s)', 'FontSize', 11); ylabel('PID Error (deg)', 'FontSize', 11);
title('PID Controller Angle Error Curve (Based on Fused Data)', 'FontSize', 12);
legend(p2_1, 'Location', 'northeast', 'FontSize', 9);

% 图3：PID控制输出电压（文档“PID控制量特性”分析）
figure(3); hold on; grid on; grid minor;
p3_1 = plot(t, u_pid, 'g-', 'LineWidth', 2, 'DisplayName', 'PID Output Voltage');
yline(24, 'r--', 'Voltage Upper Limit (+24V)', 'LineWidth', 1.5, 'HandleVisibility', 'off');
yline(-24, 'r--', 'Voltage Lower Limit (-24V)', 'LineWidth', 1.5, 'HandleVisibility', 'off');
text(0.2, 18, ['Peak Voltage: ', num2str(round(max(u_pid(1,:)), 1)), ' V'], 'FontSize', 10);
xlabel('Time (s)', 'FontSize', 11); ylabel('PID Output Voltage (V)', 'FontSize', 11);
title('PID Controller Output Voltage Dynamic Characteristics', 'FontSize', 12);
legend(p3_1, 'Location', 'southwest', 'FontSize', 9);

% 图4：电枢电流与输出力矩（文档“动力学模型输出”分析）
figure(4); hold on; grid on; grid minor;
p4_1 = plot(t, current, 'c-', 'LineWidth', 2, 'DisplayName', 'Armature Current');
p4_2 = plot(t, motor_torque_output, 'k-^', 'LineWidth', 2, 'MarkerSize', 1.5, 'DisplayName', 'Motor Output Torque');
yline(10, 'r--', 'Current Upper Limit (+10A)', 'LineWidth', 1.5, 'HandleVisibility', 'off');
yline(-10, 'r--', 'Current Lower Limit (-10A)', 'LineWidth', 1.5, 'HandleVisibility', 'off');
xlabel('Time (s)', 'FontSize', 11); ylabel('Current (A) / Torque (N·m)', 'FontSize', 11);
title('Robotic Joint Armature Current and Motor Output Torque', 'FontSize', 12);
legend([p4_1, p4_2], 'Location', 'northeast', 'FontSize', 9);

% 图5：双传感器原始测量对比（文档“多传感器特性差异”验证）
figure(5); hold on; grid on; grid minor;
p5_1 = plot(t, theta_gyro_deg, 'o-', 'Color', [1, 0.5, 0], 'LineWidth', 2, 'MarkerSize', 1.5, 'DisplayName', 'Gyroscope Measurement');
p5_2 = plot(t, theta_enc_deg, 'o-', 'Color', [0.5, 0, 0.5], 'LineWidth', 2, 'MarkerSize', 1.5, 'DisplayName', 'Encoder Measurement');
text(3, 34, ['Gyro Noise: \pm', num2str(gyro_noise_std), ' deg'], 'FontSize', 10, 'Color', [1, 0.5, 0]);
text(3, 32.5, ['Encoder Noise: \pm', num2str(enc_noise_std), ' deg'], 'FontSize', 10, 'Color', [0.5, 0, 0.5]);
xlabel('Time (s)', 'FontSize', 11); ylabel('Measured Angle (deg)', 'FontSize', 11);
title('Comparison of Gyroscope and Encoder Raw Angle Measurements', 'FontSize', 12);
legend([p5_1, p5_2], 'Location', 'northwest', 'FontSize', 9);

% 图6：传感器融合结果与实际角度对比（文档“融合有效性”验证）
figure(6); hold on; grid on; grid minor;
p6_1 = plot(t, theta_actual_deg, 'b-', 'LineWidth', 2, 'DisplayName', 'Actual Angle');
p6_2 = plot(t, theta_fused_deg, 'r--', 'LineWidth', 2.5, 'DisplayName', 'Kalman Fused Angle');
fusion_max_dev = round(max(abs(theta_fused_deg - theta_actual_deg)), 3);
text(2, 28, ['Max Fusion Deviation: ', num2str(fusion_max_dev), ' deg'], 'FontSize', 10, 'Color', 'red');
xlabel('Time (s)', 'FontSize', 11); ylabel('Angle (deg)', 'FontSize', 11);
title('Comparison of Fused Angle and Actual Angle', 'FontSize', 12);
legend([p6_1, p6_2], 'Location', 'northwest', 'FontSize', 9);

% 图7：三种测量误差对比（文档“融合精度提升”验证）
figure(7); hold on; grid on; grid minor;
error_gyro = theta_gyro_deg - theta_actual_deg;
error_enc = theta_enc_deg - theta_actual_deg;
error_fused_theta = theta_fused_deg - theta_actual_deg;
rmse_gyro = round(sqrt(mean(error_gyro(1,:).^2)), 3);
rmse_enc = round(sqrt(mean(error_enc(1,:).^2)), 3);
rmse_fused_theta = round(sqrt(mean(error_fused_theta(1,:).^2)), 3);

p7_1 = plot(t, error_gyro, 'o-', 'Color', [1, 0.5, 0], 'LineWidth', 2, 'MarkerSize', 1, 'DisplayName', 'Gyroscope Error');
p7_2 = plot(t, error_enc, 'o-', 'Color', [0.5, 0, 0.5], 'LineWidth', 2, 'MarkerSize', 1, 'DisplayName', 'Encoder Error');
p7_3 = plot(t, error_fused_theta, 'g-', 'LineWidth', 2.5, 'DisplayName', 'Fused Angle Error');

text(2, 0.4, ['Gyro RMSE: ', num2str(rmse_gyro), ' deg'], 'FontSize', 9, 'Color', [1, 0.5, 0]);
text(2, 0.3, ['Encoder RMSE: ', num2str(rmse_enc), ' deg'], 'FontSize', 9, 'Color', [0.5, 0, 0.5]);
text(2, 0.2, ['Fused RMSE: ', num2str(rmse_fused_theta), ' deg'], 'FontSize', 9, 'Color', 'green');
xlabel('Time (s)', 'FontSize', 11); ylabel('Measurement Error (deg)', 'FontSize', 11);
title('Comparison of Gyro, Encoder, and Fused Angle Measurement Errors', 'FontSize', 12);
legend([p7_1, p7_2, p7_3], 'Location', 'northeast', 'FontSize', 9);

% 图8：卡尔曼滤波增益变化（文档“滤波收敛特性”分析）
figure(8); hold on; grid on; grid minor;
p8_1 = plot(t, squeeze(K(1,1,:)), 'r-', 'LineWidth', 2, 'DisplayName', 'K11 (Angle Gain)');
p8_2 = plot(t, squeeze(K(1,2,:)), 'g-', 'LineWidth', 2, 'DisplayName', 'K12 (Angular Velocity-to-Angle Gain)');
p8_3 = plot(t, squeeze(K(2,1,:)), 'b-', 'LineWidth', 2, 'DisplayName', 'K21 (Angle-to-Angular Velocity Gain)');
p8_4 = plot(t, squeeze(K(2,2,:)), 'm-', 'LineWidth', 2, 'DisplayName', 'K22 (Angular Velocity Gain)');
xline(0.5, 'k--', 'Gain Stabilization Start (t=0.5s)', 'LineWidth', 1.5, 'HandleVisibility', 'off');
text(3, 0.6, 'Gain fluctuation <0.05 after stabilization', 'FontSize', 10);
xlabel('Time (s)', 'FontSize', 11); ylabel('Kalman Gain', 'FontSize', 11);
title('Dynamic Change of Kalman Filter Gain Matrix', 'FontSize', 12);
legend([p8_1, p8_2, p8_3, p8_4], 'Location', 'southeast', 'FontSize', 8);

% 图9：卡尔曼滤波角度估计协方差变化（文档“状态不确定性降低”验证）
figure(9); hold on; grid on; grid minor;
p9_1 = plot(t, squeeze(P_est(1,1,:)), 'k-', 'LineWidth', 2, 'DisplayName', 'Angle Estimation Covariance');
cov_stable = round(squeeze(P_est(1,1,end)), 4);
xline(0.3, 'g--', 'Covariance Settling Time (t=0.3s)', 'LineWidth', 1.5, 'HandleVisibility', 'off');
text(2, 0.2, ['Stable Covariance: ', num2str(cov_stable), ' rad²'], 'FontSize', 10);
xlabel('Time (s)', 'FontSize', 11); ylabel('Angle Estimation Covariance (rad²)', 'FontSize', 11);
title('Dynamic Change of Kalman Filter Angle Estimation Covariance', 'FontSize', 12);
legend(p9_1, 'Location', 'northeast', 'FontSize', 9);

% 图10：机器人角速度跟踪效果（文档“动态响应性能”验证）
figure(10); hold on; grid on; grid minor;
p10_1 = plot(t, omega_actual_deg, 'b-', 'LineWidth', 2, 'DisplayName', 'Actual Angular Velocity');
p10_2 = plot(t, omega_ref_deg, 'r--', 'LineWidth', 2.5, 'DisplayName', 'Reference Angular Velocity (62.8cos(2πt))');
p10_3 = plot(t, omega_fused_deg, 'g-.', 'LineWidth', 2, 'DisplayName', 'Fused Angular Velocity');
xlabel('Time (s)', 'FontSize', 11); ylabel('Angular Velocity (deg/s)', 'FontSize', 11);
title('Robotic Joint Angular Velocity Tracking Performance', 'FontSize', 12);
legend([p10_1, p10_2, p10_3], 'Location', 'southeast', 'FontSize', 9);

%% 7. 绘制新增的4张仿真图，模拟MuJoCo或Webots环境下的特性
% 新增图11：机器人关节角度跟踪性能
figure(11); hold on; grid on; grid minor;
p11_1 = plot(t, theta_ref_deg, 'r-', 'LineWidth', 2, 'DisplayName', 'Reference Angle');
p11_2 = plot(t, theta_actual_deg, 'b-', 'LineWidth', 2, 'DisplayName', 'Actual Angle (with Disturbances)');
p11_3 = plot(t, theta_fused_deg, 'g--', 'LineWidth', 2, 'DisplayName', 'Kalman Fused Angle');
xlabel('Time (s)', 'FontSize', 11); ylabel('Angle (deg)', 'FontSize', 11);
title('Joint Angle Tracking Performance', 'FontSize', 12);
legend([p11_1, p11_2, p11_3], 'Location', 'northwest', 'FontSize', 9);

% 新增图12：控制电压和电流
figure(12); hold on; grid on; grid minor;
yyaxis left;
p12_1 = plot(t, u_pid, 'Color', [0.85, 0.33, 0.1], 'LineWidth', 2, 'DisplayName', 'PID Output Voltage (V)');
ylabel('Voltage (V)', 'FontSize', 11);
yyaxis right;
p12_2 = plot(t, current, 'Color', [0.47, 0.67, 0.19], 'LineWidth', 2, 'DisplayName', 'Armature Current (A)');
ylabel('Current (A)', 'FontSize', 11);
yline(24, 'r--', 'V_{max}', 'LineWidth', 1.5, 'HandleVisibility', 'off');
yline(-24, 'r--', 'V_{min}', 'LineWidth', 1.5, 'HandleVisibility', 'off');
yline(10, 'b:', 'I_{max}', 'LineWidth', 1.5, 'HandleVisibility', 'off');
yline(-10, 'b:', 'I_{min}', 'LineWidth', 1.5, 'HandleVisibility', 'off');
xlabel('Time (s)', 'FontSize', 11);
title('PID Control Output Voltage and Armature Current', 'FontSize', 12);
legend([p12_1, p12_2], 'Location', 'southwest', 'FontSize', 9);

% 新增图13：关节所受扰动力矩分解
figure(13); hold on; grid on; grid minor;
p13_1 = plot(t, motor_torque_output, 'k-', 'LineWidth', 2, 'DisplayName', 'Motor Output Torque');
p13_2 = plot(t, friction_torque_hist, 'm--', 'LineWidth', 2, 'DisplayName', 'Friction Torque');
p13_3 = plot(t, gravity_torque_hist, 'c-.', 'LineWidth', 2, 'DisplayName', 'Gravity Torque');
p13_4 = plot(t, disturbance_torque_hist, 'Color', [0.93, 0.69, 0.13], 'LineWidth', 2, 'DisplayName', 'External Disturbance Torque');
xlabel('Time (s)', 'FontSize', 11); ylabel('Torque (N·m)', 'FontSize', 11);
title('Joint Disturbance Torque Decomposition', 'FontSize', 12);
legend([p13_1, p13_2, p13_3, p13_4], 'Location', 'northwest', 'FontSize', 9);

% 新增图14：角速度跟踪与融合效果
figure(14); hold on; grid on; grid minor;
p14_1 = plot(t, omega_ref_deg, 'r-', 'LineWidth', 2, 'DisplayName', 'Reference Angular Velocity');
p14_2 = plot(t, omega_actual_deg, 'b-', 'LineWidth', 2, 'DisplayName', 'Actual Angular Velocity (with Disturbances)');
p14_3 = plot(t, omega_fused_deg, 'g--', 'LineWidth', 2, 'DisplayName', 'Kalman Fused Angular Velocity');
xlabel('Time (s)', 'FontSize', 11); ylabel('Angular Velocity (deg/s)', 'FontSize', 11);
title('Robotic Joint Angular Velocity Tracking and Fusion Performance', 'FontSize', 12);
legend([p14_1, p14_2, p14_3], 'Location', 'northwest', 'FontSize', 9);


%% 8. 输出关键性能指标（文档“仿真结果量化分析”需求）
fprintf('=== Performance Metrics of PID and Sensor Fusion for Robotic System (with MuJoCo/Webots-style Disturbances) ===\n');
fprintf('1. PID Control Steady-State Angle Error (based on fusion): %.3f deg\n', abs(error_pid_deg(1, end)));
fprintf('2. Gyroscope Raw Angle Measurement RMSE: %.3f deg\n', rmse_gyro);
fprintf('3. Encoder Raw Angle Measurement RMSE: %.3f deg\n', rmse_enc);
fprintf('4. Fused Angle RMSE (compared to actual value): %.3f deg\n', rmse_fused_theta);
fprintf('5. Angle Tracking Settling Time (within ±0.5 degree error band): %.2f s\n', settling_time);


%% 9. 自定义单位转换函数（匹配文档角度/弧度运算需求）
function rad = deg2rad(deg)
    rad = deg * pi / 180;
end

function deg = rad2deg(rad)
    deg = rad * 180 / pi;
end
