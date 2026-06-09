% UR10 实体 3D 仿真：沿末端 Z 轴纯直线运动 (修复末端连杆缺失)
clear; clc; close all;

%% 1. 轨迹规划参数设置
P_start = [500; 0; 400];  
R_target = [0, 0, 1; 1, 0, 0; 0, 1, 0];
dir_z = R_target(:, 3); 

travel_distance = 350; 
t = linspace(0, 1, 60); 
travel = travel_distance * t; 

trace_points = []; 

%% 2. 仿真环境初始化
figure('Name', 'UR10 3D Solid Simulation', 'Color', 'w');
view(3); grid on; hold on;
axis([-500 1000 -500 500 -100 1000]);
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('UR10 实体三维渲染：纯直线运动');

camlight('headlight'); 
camlight('left');
lighting gouraud;
material dull; 

X_ref = P_start(1) + dir_z(1) * [0, travel_distance];
Y_ref = P_start(2) + dir_z(2) * [0, travel_distance];
Z_ref = P_start(3) + dir_z(3) * [0, travel_distance];
plot3(X_ref, Y_ref, Z_ref, '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 2);

%% 3. 轨迹跟踪与可视化
for i = 1:length(t)
    P_ee = P_start + dir_z * travel(i);
    T_target = [R_target, P_ee; 0 0 0 1];
    
    theta = ur10_inverse_kinematics(T_target);
    [joint_positions, T_actual] = ur10_forward_kinematics(theta);
    
    p_current = joint_positions(:, end);
    trace_points = [trace_points, p_current];
    
    cla; 
    plot3(X_ref, Y_ref, Z_ref, '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 2);
    plot3(trace_points(1,:), trace_points(2,:), trace_points(3,:), 'k-', 'LineWidth', 2);
    
    % ================= 绘制 3D 实体机械臂 =================
    link_color = [0.2 0.5 0.8]; 
    joint_color = [0.3 0.3 0.3]; 
    
    % 修复：循环上限改为 7，遍历所有点和连杆
    for j = 1:7
        draw_sphere(joint_positions(:, j), 45, joint_color);
        if j < 7
            draw_cylinder(joint_positions(:, j), joint_positions(:, j+1), 35, link_color);
        end
    end
    % =====================================================
    
    R_current = T_actual(1:3, 1:3); 
    axis_length = 80; 
    
    quiver3(p_current(1), p_current(2), p_current(3), R_current(1,1)*axis_length, R_current(2,1)*axis_length, R_current(3,1)*axis_length, 0, 'r', 'LineWidth', 3, 'MaxHeadSize', 0.5);
    quiver3(p_current(1), p_current(2), p_current(3), R_current(1,2)*axis_length, R_current(2,2)*axis_length, R_current(3,2)*axis_length, 0, 'g', 'LineWidth', 3, 'MaxHeadSize', 0.5);
    quiver3(p_current(1), p_current(2), p_current(3), R_current(1,3)*axis_length, R_current(2,3)*axis_length, R_current(3,3)*axis_length, 0, 'b', 'LineWidth', 3, 'MaxHeadSize', 0.5);
        
    drawnow;
end

%% ===== 局部函数：绘制 3D 圆柱连杆 =====
function draw_cylinder(p1, p2, r, col)
    dir_vec = p2 - p1;
    L = norm(dir_vec);
    if L < 1e-4, return; end
    dir_vec = dir_vec / L; 
    
    [X, Y, Z] = cylinder(r, 30);
    Z = Z * L; 
    
    v0 = [0; 0; 1];
    v = cross(v0, dir_vec);
    s = norm(v);
    c = dot(v0, dir_vec);
    if s < 1e-6
        if c > 0, R = eye(3); else, R = diag([1, -1, -1]); end
    else
        vx = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
        R = eye(3) + vx + vx^2 * ((1-c)/(s^2)); 
    end
    
    [m, n] = size(X);
    for i = 1:m
        for j = 1:n
            pt = R * [X(i,j); Y(i,j); Z(i,j)] + p1;
            X(i,j) = pt(1); Y(i,j) = pt(2); Z(i,j) = pt(3);
        end
    end
    
    surf(X, Y, Z, 'FaceColor', col, 'EdgeColor', 'none', 'BackFaceLighting', 'lit');
end

%% ===== 局部函数：绘制 3D 关节球体 =====
function draw_sphere(p, r, col)
    [X, Y, Z] = sphere(30);
    X = X * r + p(1); Y = Y * r + p(2); Z = Z * r + p(3);
    surf(X, Y, Z, 'FaceColor', col, 'EdgeColor', 'none', 'BackFaceLighting', 'lit');
end

%% ===== 逆解与正解函数 =====
function theta = ur10_inverse_kinematics(T)
    n = T(1:3, 1);  o = T(1:3, 2);  a = T(1:3, 3); p = T(1:3, 4);
    nx=n(1); ny=n(2); nz=n(3); ox=o(1); oy=o(2); oz=o(3); ax=a(1); ay=a(2); az=a(3); px=p(1); py=p(2); pz=p(3);
    d1=114; a2=400; a3=300; d4=165.4; d5=136; d6=233.5;
    
    P6 = p - d6 * a; 
    val1 = d4 / sqrt(P6(1)^2 + P6(2)^2); val1 = max(min(val1, 1), -1); 
    theta1 = atan2(P6(2), P6(1)) + acos(val1) + pi/2; 
    c1 = cos(theta1); s1 = sin(theta1);
    
    val5 = (px * s1 - py * c1 - d4) / d6; val5 = max(min(val5, 1), -1);
    theta5 = acos(val5); 
    c5 = cos(theta5); s5 = sin(theta5);
    
    if abs(s5) < 1e-6, theta6 = 0; else, theta6 = atan2((-ox*s1 + oy*c1)/s5, (nx*s1 - ny*c1)/s5); end
    c6 = cos(theta6); s6 = sin(theta6);
    
    P4x1 = -d6*(ax*c1 + ay*s1) + d5*(ox*c1*c6 + oy*s1*c6 + nx*c1*s6 + ny*s1*s6) + px*c1 + py*s1;
    P4z1 = -d6*az + d5*(oz*c6 + nz*s6) + pz - d1;
    
    val3 = (P4x1^2 + P4z1^2 - a2^2 - a3^2) / (2 * a2 * a3); val3 = max(min(val3, 1), -1);
    theta3 = -acos(val3); 
    c3 = cos(theta3); s3 = sin(theta3);
    
    theta2 = atan2((a3*c3 + a2)*P4z1 - a3*s3*P4x1, (a3*c3 + a2)*P4x1 + a3*s3*P4z1);
    
    n4x1 = -s5*(ax*c1 + ay*s1) + c5*(nx*c1*c6 + ny*s1*c6 - ox*c1*s6 - oy*s1*s6);
    n4z1 = nz*c5*c6 - oz*c5*s6 - az*s5;
    theta4 = atan2(n4z1, n4x1) - theta2 - theta3;
    
    theta = [theta1, theta2, theta3, theta4, theta5, theta6];
end

function [joint_positions, T] = ur10_forward_kinematics(theta)
    d1=114; a2=400; a3=300; d4=165.4; d5=136; d6=233.5;
    DH = [ 0 0 d1 theta(1); pi/2 0 0 theta(2); 0 a2 0 theta(3); 0 a3 d4 theta(4); pi/2 0 d5 theta(5); -pi/2 0 d6 theta(6)];
    joint_positions = zeros(3, 7); T = eye(4);
    for i = 1:6
        alpha = DH(i, 1); a = DH(i, 2); d = DH(i, 3); th = DH(i, 4);
        Rx = [1 0 0 0; 0 cos(alpha) -sin(alpha) 0; 0 sin(alpha) cos(alpha) 0; 0 0 0 1];
        Dx = [1 0 0 a; 0 1 0 0; 0 0 1 0; 0 0 0 1];
        Rz = [cos(th) -sin(th) 0 0; sin(th) cos(th) 0 0; 0 0 1 0; 0 0 0 1];
        Dz = [1 0 0 0; 0 1 0 0; 0 0 1 d; 0 0 0 1];
        T = T * (Rx * Dx * Rz * Dz); joint_positions(:, i+1) = T(1:3, 4);
    end
end