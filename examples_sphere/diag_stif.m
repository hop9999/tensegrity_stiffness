clc; clear; close all

robot = RobotSetup();
act_rod = [4,4,3,3];
act_node = [1,2,1,2];

load stiff_active_springs100.mat

e = [-0.0001; 0.0001; 0.0001];
K_des = [H_tensor(1,1,:),H_tensor(2,2,:),H_tensor(3,3,:)];
K_des = reshape(K_des,3,4);
[U,S,V] = svd(K_des) 
Un = U(:,1:2);
en = Un*Un'*e

delta = -pinv(K_des)*e

f = -1;
robot.l(10) = robot.l(10) + f*delta(1);
robot.l(12) = robot.l(12) + f*delta(2);
robot.l(19) = robot.l(19) + f*delta(3);
robot.l(23) = robot.l(23) + f*delta(4);

fmincon_options = optimoptions('fmincon','Display','notify','Algorithm','interior-point','MaxIterations',...
    150000,'MaxFunctionEvaluations',150000,'OptimalityTolerance', 10e-16,...
    'ConstraintTolerance', 10e-16,'StepTolerance', 10e-16);

K01 = get_full_stiffness_matrix_Cartesian(robot, 'act_rods',       act_rod, ...
                                                 'act_nodes',      act_node, ...
                                                 'fmincon_options', fmincon_options);
(K01(1:3,1:3)-K0(1:3,1:3))