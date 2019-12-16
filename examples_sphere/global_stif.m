clc; clear; close all

robot = RobotSetup();

act_rod = [4,4,3,3];
act_node = [1,2,1,2];

fmincon_options = optimoptions('fmincon','Display','notify','Algorithm','interior-point','MaxIterations',...
    150000,'MaxFunctionEvaluations',150000,'OptimalityTolerance', 10e-16,...
    'ConstraintTolerance', 10e-16,'StepTolerance', 10e-16);

x = forward_kinematics(robot, 'fmincon_options', fmincon_options);
p = end_effector_position_from_state(robot, x);

% dl = 0.01;
K0 = get_full_stiffness_matrix_Cartesian(robot, 'act_rods',       act_rod, ...
                                                 'act_nodes',      act_node)

H_tensor = get_full_stiffness_tensor_Cartesian(robot, 'active_springs', robot.active_springs,...
                                                      'act_rods',       act_rod, ...
                                                      'act_nodes',      act_node);
save stiff_active_springs100.mat K0 H_tensor