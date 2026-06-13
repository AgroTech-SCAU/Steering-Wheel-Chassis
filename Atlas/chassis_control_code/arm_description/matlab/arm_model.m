clear;
clc;
close all;

% 坐标系约定：
% +X 朝前
% +Y 朝左
% +Z 朝天
%
% 关节约定：
% q0：底座 yaw，轴朝天，右手法则，朝前为 0°
% q1：肩 pitch，等效轴朝右，右手法则，朝天为 0°
% q2：肘 pitch，等效轴朝右，右手法则，朝天为 0°
% q3：腕 pitch，等效轴朝右，右手法则，朝天为 0°
% q4：末端 pitch，按 Atlas.urdf 的末端轴等效
%
% 说明：这里使用 MDH
% 注意：本文件只建 Atlas.urdf 中 arm0~arm4 机械臂链；
%       不计入底盘 base_link -> arm0 的 [0.0300, 0, 0.2051] 平移。
%       输出位姿相对 arm_base，arm_base 的轴方向与 base_link 一致。

% ========================= 几何信息，单位 m ========================= %

% Atlas.urdf 机械臂关节原点：
% arm0: base_link -> arm0_Link, xyz = [ 0.03000000,  0.00000000,  0.20510000], rpy = [0,        0,          0],          axis = [0, 0,  1]
% arm1: arm0_Link -> arm1_Link, xyz = [ 0.02762005,  0.01626790,  0.06050000], rpy = [pi/2,     0,          0],          axis = [0, 0,  1]
% arm2: arm1_Link -> arm2_Link, xyz = [-0.21672413,  0.00000000,  0.01920686], rpy = [0,        0,          0],          axis = [0, 0, -1]
% arm3: arm2_Link -> arm3_Link, xyz = [ 0.19444619,  0.04799841,  0.01712735], rpy = [0.008727,-0.008726, -0.000076], axis ≈ [0, 0, -1]
% arm4: arm3_Link -> arm4_Link, xyz = [ 0.04426412, -0.02032989, -0.01877657], rpy = [1.570398, 0.008386, -0.046675], axis ≈ [0, 0, -1]

BASE_X = 0.0;
BASE_Y = 0.0;

% arm0 yaw 轴心到 arm1 shoulder pitch 的高度
H0 = 0.0605000000000000;

% 由 Atlas.urdf 的 arm0~arm4 关节轴线等效得到的 MDH 尺寸
A1_offset = 0.0276200491203067;
D1_offset = -0.0162679040568649;

L1_len = 0.2167241256700170;
D2_offset = -0.0192068569153542;

L2_len = 0.2002827243995208;
D3_offset = 0.0014389528584892;

L3_len = 0.0451594898594991;

% 末端固定变换：从最后一个 MDH 坐标系到 Atlas.urdf 的 arm4_Link
T_tool = [
    0.9999619259637,  -0.0000000000000,  -0.0087262032439,   0.0000000000000;
    0.0000761495224,  -0.9999619230642,   0.0087262032439,   0.0000000000000;
   -0.0087258709769,  -0.0087265354984,  -0.9999238504776,  -0.0184685931641;
    0.0000000000000,   0.0000000000000,   0.0000000000000,   1.0000000000000
];

% ========================= 统一 MDH 参数建模 ========================= %

% 这组参数沿用前面已验证与 Atlas.urdf FK 等效的 MDH 几何。
% 仅把默认舵机零位改为 [180 180 180 180 180]，使全 180° 时机械臂朝天。

L0 = Link('alpha', 0,      'a', 0,         'offset', -pi,                    'd', H0,        'modified');
L1 = Link('alpha', -pi/2,   'a', A1_offset, 'offset', pi/2,                  'd', D1_offset, 'modified');
L2 = Link('alpha', pi,    'a', L1_len,    'offset', 3.3836013435535577,    'd', D2_offset, 'modified');
L3 = Link('alpha', 0,      'a', L2_len,    'offset', 2.8616351199480290,    'd', D3_offset, 'modified');
L4 = Link('alpha', pi/2,   'a', L3_len,    'offset', -pi,                    'd', 0,         'modified');

% 输入方向修正
L0.flip = true;
L2.flip = true;

servo_arm = SerialLink([L0 L1 L2 L3 L4], 'name', 'atlas_arm_mdh_180_upright_exact');
servo_arm.base = transl(BASE_X, BASE_Y, 0);
servo_arm.tool = T_tool;

% ========================= 关节限位 ========================= %

servo_arm.links(1).qlim = deg2rad([0, 360]); % q0 yaw
servo_arm.links(2).qlim = deg2rad([0, 360]); % q1 shoulder
servo_arm.links(3).qlim = deg2rad([0, 360]); % q2 elbow
servo_arm.links(4).qlim = deg2rad([0, 360]); % q3 wrist
servo_arm.links(5).qlim = deg2rad([0, 360]); % q4 end

% ========================= 零位验证与默认显示姿态 ========================= %

q_mdh_zero_deg = [180, 90, 360, 180, 180];
q_mdh_zero = deg2rad(q_mdh_zero_deg);

T_mdh_zero = servo_arm.fkine(q_mdh_zero);
T_mdh_zero_mat = double(T_mdh_zero);
R_end = T_mdh_zero_mat(1:3, 1:3);
p_end = T_mdh_zero_mat(1:3, 4);

disp('舵机零位 q_mdh_zero_deg = [180 90 360 180 180] 时的末端位姿 T_mdh_zero = ');
disp(T_mdh_zero);

disp('舵机零位末端位置 transl(T_mdh_zero) = ');
disp(transl(T_mdh_zero));

disp('MDH 参数表 [alpha, a, offset, d] = ');
disp([
    0,      0,         -pi,                  H0;
    -pi/2,   A1_offset, pi/2,                D1_offset;
   -pi,     L1_len,    3.3836013435535577,  D2_offset;
    0,      L2_len,    -2.8616351199480290,  D3_offset;
    pi/2,   L3_len,    -pi,                  0
]);

disp('base_link / arm_base axis，表达在 arm_base 中：');
fprintf('+X = [%.6f %.6f %.6f] 朝前\n', 1, 0, 0);
fprintf('+Y = [%.6f %.6f %.6f] 朝左\n', 0, 1, 0);
fprintf('+Z = [%.6f %.6f %.6f] 朝天\n', 0, 0, 1);

disp('末端 axis，表达在 arm_base 中：');
fprintf('+X = [%.6f %.6f %.6f]\n', R_end(:, 1));
fprintf('+Y = [%.6f %.6f %.6f]\n', R_end(:, 2));
fprintf('+Z = [%.6f %.6f %.6f]\n', R_end(:, 3));

servo_zero_deg = [180, 90, 360, 180, 180];
q_default = deg2rad(servo_zero_deg);

T_default = servo_arm.fkine(q_default);

disp('默认显示的舵机零位 servo_zero_deg = ');
disp(servo_zero_deg);

disp('默认显示零位 q_default(rad) = ');
disp(q_default);

disp('默认显示零位末端位姿 T_default = ');
disp(T_default);

disp('默认显示零位末端位置 transl(T_default) = ');
disp(transl(T_default));

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

% 零位姿态(°): 180; 180; 180; 180; 180
% 零位姿态(rad): pi; pi; pi; pi; pi
