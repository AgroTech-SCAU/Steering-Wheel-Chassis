clear;
clc;
close all;

% 说明：这里使用 MDH
% 本文件只建 Atlas.urdf 中 arm0~arm4 机械臂链；
% 不计入底盘 base_link -> arm0 的 [0.0300, 0, 0.2051] 平移
% 输出位姿相对 arm_base，arm_base 的轴方向与 base_link 一致

% ========================= 几何信息，单位 m ========================= %

% Atlas.urdf 机械臂关节原点：
% arm0: base_link -> arm0_Link, xyz = [ 0.03000000,  0.00000000,  0.20510000], rpy = [0,        0,          0],          axis = [0, 0,  1]
% arm1: arm0_Link -> arm1_Link, xyz = [ 0.02762005,  0.01626790,  0.06050000], rpy = [pi/2,     0,          0],          axis = [0, 0,  1]
% arm2: arm1_Link -> arm2_Link, xyz = [-0.21672413,  0.00000000,  0.01920686], rpy = [0,        0,          0],          axis = [0, 0, -1]
% arm3: arm2_Link -> arm3_Link, xyz = [ 0.19444619,  0.04799841,  0.01712735], rpy = [0.008727,-0.008726, -0.000076], axis ≈ [0, 0, -1]
% arm4: arm3_Link -> arm4_Link, xyz = [ 0.04426412, -0.02032989, -0.01877657], rpy = [1.570398, 0.008386, -0.046675], axis ≈ [0, 0, -1]

BASE_X = 0.0;
BASE_Y = 0.0;
BASE_Z = 0.0605000000000000;

A0 = 0.0000000000000000;
A1 = 0.0276200491203067;
A2 = 0.2167241256700170;
A3 = 0.2002827243995208;
A4 = 0.0451594898594991;

D0 = 0.0000000000000000;
D1 = -0.0162679040568649;
D2 = -0.0192068569153542;
D3 = 0.0014389528584892;
D4 = 0.0000000000000000;

TCP_len = 0.0;

% 舵机到 URDF 角的标定关系：
% SERVO_HOME_DEG 对应 URDF 关节角 [0 0 0 0 0]
% 注意：这个不是默认显示姿态；默认显示姿态在后面的 servo_zero_deg 设置
SERVO_HOME_DEG = [180, 90, 360, 180, 180];

% ========================= 统一 MDH 参数建模 ========================= %

L0 = Link('alpha', 0,      'a', A0, 'offset',  -pi,                  'd', D0, 'modified');
L1 = Link('alpha', pi/2,   'a', A1, 'offset',  -pi/2,                'd', D1, 'modified');
L2 = Link('alpha', pi,     'a', A2, 'offset',  -3.3836013435535577,  'd', D2, 'modified');
L3 = Link('alpha', 0,      'a', A3, 'offset',  -2.8616351199480290,  'd', D3, 'modified');
L4 = Link('alpha', pi/2,   'a', A4, 'offset',  -pi,                  'd', D4, 'modified');

% 输入方向修正
L0.flip = true;
L1.flip = true;
L4.flip = true;

servo_arm = SerialLink([L0 L1 L2 L3 L4], 'name', 'atlas_arm0_to_arm4_mdh_exact');
servo_arm.base = transl(BASE_X, BASE_Y, BASE_Z);
servo_arm.tool = [
    0.9999619259637,  -0.0000000000000,  -0.0087262032439,   0.0000000000000;
    0.0000761495224,  -0.9999619230642,   0.0087262032439,   0.0000000000000;
   -0.0087258709769,  -0.0087265354984,  -0.9999238504776,  -0.0184685931641;
    0.0000000000000,   0.0000000000000,   0.0000000000000,   1.0000000000000
];

% ========================= 关节限位 ========================= %

servo_arm.links(1).qlim = deg2rad([0, 360]); % q0 yaw
servo_arm.links(2).qlim = deg2rad([0, 360]); % q1 shoulder
servo_arm.links(3).qlim = deg2rad([0, 360]); % q2 elbow
servo_arm.links(4).qlim = deg2rad([0, 360]); % q3 wrist
servo_arm.links(5).qlim = deg2rad([0, 360]); % q4 end

% ========================= 零位验证与默认显示姿态 ========================= %

q_home_deg = SERVO_HOME_DEG;
q_home = deg2rad(q_home_deg);
T_home = servo_arm.fkine(q_home);

disp('标定 home 姿态 q_home_deg = [180 90 360 180 180]，对应 URDF 关节角 [0 0 0 0 0]：');
disp(T_home);
disp('home 末端位置 transl(T_home) = ');
disp(transl(T_home));

disp('MDH 参数表 [alpha, a, offset, d] = ');
disp([
    0,      A0,  pi,                  D0;
    pi/2,   A1, -pi/2,                D1;
   -pi,     A2, -3.3836013435535577,  D2;
    0,      A3, -2.8616351199480290,  D3;
    pi/2,   A4,  pi,                  D4
]);

disp('base_link / arm_base axis，表达在 arm_base 中：');
fprintf('+X = [%.6f %.6f %.6f] 朝前\n', 1, 0, 0);
fprintf('+Y = [%.6f %.6f %.6f] 朝左\n', 0, 1, 0);
fprintf('+Z = [%.6f %.6f %.6f] 朝天\n', 0, 0, 1);

servo_zero_deg = [180, 90, 360, 180, 180];
q_default = deg2rad(servo_zero_deg);

T_default = servo_arm.fkine(q_default);
T_default_mat = double(T_default);
R_end = T_default_mat(1:3, 1:3);
p_end = T_default_mat(1:3, 4);

disp('默认显示的舵机零位 servo_zero_deg = [180 90 360 180 180]：');
disp(servo_zero_deg);

disp('默认显示零位 q_default(rad) = ');
disp(q_default);

disp('默认显示零位末端位姿 T_default = ');
disp(T_default);

disp('默认显示零位末端位置 transl(T_default) = ');
disp(transl(T_default));

disp('末端 axis，表达在 arm_base 中：');
fprintf('+X = [%.6f %.6f %.6f]\n', R_end(:, 1));
fprintf('+Y = [%.6f %.6f %.6f]\n', R_end(:, 2));
fprintf('+Z = [%.6f %.6f %.6f]\n', R_end(:, 3));

figure;
servo_arm.plot(q_default, ...
    'workspace', [-0.45 0.45 -0.45 0.45 -0.05 0.65], ...
    'scale', 0.6);

hold on;
axis_len = 0.06;

% base_link / arm_base axis
quiver3(0, 0, 0, axis_len, 0, 0, 'r', 'LineWidth', 2);
quiver3(0, 0, 0, 0, axis_len, 0, 'g', 'LineWidth', 2);
quiver3(0, 0, 0, 0, 0, axis_len, 'b', 'LineWidth', 2);
text(axis_len, 0, 0, 'base +X');
text(0, axis_len, 0, 'base +Y');
text(0, 0, axis_len, 'base +Z');

% end axis
quiver3(p_end(1), p_end(2), p_end(3), axis_len*R_end(1,1), axis_len*R_end(2,1), axis_len*R_end(3,1), 'r', 'LineWidth', 2);
quiver3(p_end(1), p_end(2), p_end(3), axis_len*R_end(1,2), axis_len*R_end(2,2), axis_len*R_end(3,2), 'g', 'LineWidth', 2);
quiver3(p_end(1), p_end(2), p_end(3), axis_len*R_end(1,3), axis_len*R_end(2,3), axis_len*R_end(3,3), 'b', 'LineWidth', 2);
text(p_end(1)+axis_len*R_end(1,1), p_end(2)+axis_len*R_end(2,1), p_end(3)+axis_len*R_end(3,1), 'end +X');
text(p_end(1)+axis_len*R_end(1,2), p_end(2)+axis_len*R_end(2,2), p_end(3)+axis_len*R_end(3,2), 'end +Y');
text(p_end(1)+axis_len*R_end(1,3), p_end(2)+axis_len*R_end(2,3), p_end(3)+axis_len*R_end(3,3), 'end +Z');

xlabel('+X forward');
ylabel('+Y left');
zlabel('+Z up');
grid on;
axis equal;
view(135, 25);

servo_arm.teach(q_default);
servo_arm.display();

% 正解：
%   T = servo_arm.fkine(q);
%
% 逆解：
%   q_seed = q_default;
%   q_sol = servo_arm.ikcon(T_target, q_seed);
%
% 说明：
%   SERVO_HOME_DEG = [180 90 360 180 180] 只用于保证 MDH 与 Atlas.urdf 的关节角映射一致；
%   servo_zero_deg = [180 90 360 180 180] 是实体默认显示/朝天姿态
