clc; clear; close all

robot = RobotSetup();

act_rod = [4,4,3,3];
act_node = [1,2,1,2];

fmincon_options = optimoptions('fmincon','Display','notify','Algorithm','interior-point','MaxIterations',...
    150000,'MaxFunctionEvaluations',150000,'OptimalityTolerance', 10e-16,...
    'ConstraintTolerance', 10e-16,'StepTolerance', 10e-16);

n = 10
dl = linspace(-0.05,0.05,n)
C = zeros(3,n);
for i = 1:n
    robot.l(10) = robot.l(10) + dl(i);
    robot.l(10)
    K0 = get_full_stiffness_matrix_Cartesian(robot, 'act_rods',       act_rod, ...
                                                 'act_nodes',      act_node,...
                                                 'fmincon_options', fmincon_options)
    C(:,i) = K0(1,1:3)'
    robot.l(10) = robot.l(10) - dl(i);
end
C(:,i)
hold on
plot(C(1,:))
plot(C(2,:))
plot(C(3,:))