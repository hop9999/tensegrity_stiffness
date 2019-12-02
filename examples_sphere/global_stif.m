clc; clear; close all

robot = RobotSetup();

act_rod = [3,3,4,4];
act_node = [1,2,1,2];

fmincon_options = optimoptions(@fmincon,'MaxIterations', 5000, 'MaxFunctionEvaluations', 5000, 'Display', 'off');
x = forward_kinematics(robot, 'fmincon_options', fmincon_options);
p = end_effector_position_from_state(robot, x);

% dl = 0.01;
% K0 = get_full_stiffness_matrix_Cartesian(robot, 'act_rods',       act_rod, ...
%                                                 'act_nodes',      act_node);

H_tensor = get_full_stiffness_tensor_Cartesian(robot, 'active_springs', robot.active_springs,...
                                                      'act_rods',       act_rod, ...
                                                      'act_nodes',      act_node);
