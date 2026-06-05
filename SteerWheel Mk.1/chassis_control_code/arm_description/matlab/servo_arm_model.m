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
% q4：末端 pitch，等效轴朝右，右手法则，朝天为 0°
%
% 说明：这里使用 MDH

% ========================= 几何信息，单位 m ========================= %

% URDF 关节原点：
% joint1: base_link -> link1, xyz = [-0.01000,      0.00000,    0.04800],   rpy = [0,       0,  0]
% joint2: link1     -> link2, xyz = [-0.01391,      0.00390,    0.06028],   rpy = [1.5708,  0,  0]
% joint3: link2     -> link3, xyz = [ 0.00000,      0.18000,    -0.0001],   rpy = [0,       0,  0]
% joint4: link3     -> link4, xyz = [ 0.00000,      0.14710,    0.00000],   rpy = [0,       0,  0]
% joint5: link4     -> link5, xyz = [-0.00060707,   0.14000,    0.00000],   rpy = [0,       0,  0]

BASE_X = -0.01000 - 0.01391;
BASE_Y =  0.00390;

H0 = 0.04800 + 0.06028;

L1_len = hypot(0.1800, 0.0001);
L2_len = 0.1471;
L3_len = hypot(0.1400, 0.00060707);

TCP_len = 0.0;

% ========================= 统一 MDH 参数建模 ========================= %

L0 = Link('alpha', 0,      'a', 0,      'offset', 0,      'd', H0,     'modified');
L1 = Link('alpha', pi/2,   'a', 0,      'offset', pi/2,   'd', 0,      'modified');
L2 = Link('alpha', 0,      'a', L1_len, 'offset', 0,      'd', 0,      'modified');
L3 = Link('alpha', 0,      'a', L2_len, 'offset', 0,      'd', 0,      'modified');
L4 = Link('alpha', 0,      'a', L3_len, 'offset', 0,      'd', 0,      'modified');

% 关节3和关节5输入方向修正
L2.flip = true;
L4.flip = true;

servo_arm = SerialLink([L0 L1 L2 L3 L4], 'name', 'servo_5_joint_arm_mdh_from_urdf');
servo_arm.base = transl(BASE_X, BASE_Y, 0);
servo_arm.tool = transl(TCP_len, 0, 0);

% ========================= 关节限位 ========================= %

servo_arm.links(1).qlim = [-3*pi/4, 3*pi/4]; % q0 yaw
servo_arm.links(2).qlim = [-3*pi/4, 3*pi/4]; % q1 shoulder
servo_arm.links(3).qlim = [-3*pi/4, 3*pi/4]; % q2 elbow
servo_arm.links(4).qlim = [-3*pi/4, 3*pi/4]; % q3 wrist1
servo_arm.links(5).qlim = [-3*pi/4, 3*pi/4]; % q4 wrist2

% ========================= 零位验证与默认显示姿态 ========================= %

q_mdh_zero = zeros(1, 5);

T_mdh_zero = servo_arm.fkine(q_mdh_zero);

expected_z = H0 + L1_len + L2_len + L3_len + TCP_len;
expected_pos = [BASE_X, BASE_Y, expected_z];

disp('数学 MDH 零位 q_mdh_zero = [0 0 0 0 0] 时的末端位姿 T_mdh_zero = ');
disp(T_mdh_zero);

disp('数学 MDH 零位末端位置 transl(T_mdh_zero) = ');
disp(transl(T_mdh_zero));

disp('按 URDF 主尺寸估计的数学 MDH 零位末端位置 expected_pos = ');
disp(expected_pos);

disp('MDH 参数表 [alpha, a, offset, d] = ');
disp([
    0,      0,      0,      H0;
    pi/2,   0,      pi/2,   0;
    0,      L1_len, 0,      0;
    0,      L2_len, 0,      0;
    0,      L3_len, 0,      0
]);

servo_zero_deg = [0, 108, 135, -81, 0];
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
    'workspace', [-0.45 0.45 -0.45 0.45 0 0.75], ...
    'scale', 0.6);

servo_arm.teach(q_default);
servo_arm.display();

% 零位姿态(°): 0; 108; 135; -81; 0
% 零位姿态(rad): 0; 1.884; 2.335; -1.413; 0
