clc; clear; close all

robot = RobotSetup();

[mean_v,min_v,max_v] = experiment_statistics_stiffness_linearization(robot,1)

function error = get_relative_stiffness_linearization_error(robot, act_rod, act_node, alpha)
fmincon_options = optimoptions(@fmincon,'MaxIterations', 5000, 'MaxFunctionEvaluations', 5000, 'Display', 'off');
x = forward_kinematics(robot, 'fmincon_options', fmincon_options);
K0 = get_full_stiffness_matrix_Cartesian(robot, 'act_rods',       act_rod, ...
                                                'act_nodes',      act_node);
external_force = randn(3,1);
external_force = external_force/norm(external_force);

delta_X_theor = K0*[external_force;zeros(9,1)]*alpha;
robot.f = alpha*external_force;
robot.end_eff.rod = act_rod(1);
robot.end_eff.end = act_node(1);
x1 = forward_kinematics(robot, 'fmincon_options', fmincon_options);
delta_X_sim = x1 - x;
error = norm(delta_X_sim - delta_X_theor')/norm(delta_X_sim);
end

function [mean_v,min_v,max_v] = experiment_statistics_stiffness_linearization(robot,alpha)
    act_rod = [3,3,4,4];
    act_node = [1,2,1,2];
    experiment_number = 1;
    error = zeros(experiment_number,1);
    for i = 1:experiment_number
        error(i) = get_relative_stiffness_linearization_error(robot, act_rod, act_node, alpha);
    end
    mean_v = mean(error);
    min_v = min(error);
    max_v = max(error);
end