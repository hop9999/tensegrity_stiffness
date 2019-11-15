clc
clear all
close all

d1 = 1;
robot.k = ones(1,24)*500;
robot.d = [d1,d1,d1,d1,d1,d1];
robot.l = ones(1,24)*0.3;
robot.x0 = [0.3,-0.5,0.3,0.3,0.5,0.3,...
    0.3,-0.5,0.6,0.3,0.5,0.6,...
    -0.3,0.3,0.5,0.7,0.3,0.5,...
    -0.3,-0.3,0.5,0.7,-0.3,0.5];
robot.m = [1,1,1,1,1,1]*0;
robot.f = [0;0;10]*0;
robot.active_springs = [10,12,19,23];
robot.end_eff.rod = 4;
robot.end_eff.end = 1;

robot.l(10) = robot.l(10) +0.0029 ;
robot.l(12) = robot.l(12) -0.0004;
robot.l(19) = robot.l(19) +0.0003;
robot.l(23) = robot.l(23) +0.0019;

robot.base = [0,0,0,...
    0,0,1,...
    0.4,0,0,...
    0.4,0,1];
robot.planes_4_d = [];

robot.energy_f = @energy_sphere;
robot.fminc_options = optimoptions(@fmincon,'MaxIterations',5000,'MaxFunctionEvaluations',5000,'Display','off');
[p,x2] = forward_kin_tensegrity(robot);

act_rod = [3,3,4,4];
act_node = [1,2,1,2];
dl = 0.01;
K0 = cartesian_full_stiffness_tensegrity(p,robot,act_rod,act_node)

K_tens = get_k_tens(robot, robot.active_springs)

function K_tens = get_k_tens(robot, active_spr)
[p,x2] = forward_kin_tensegrity(robot);
robot.active_springs = active_spr;
act_rod = [3,3,4,4];
act_node = [1,2,1,2];
dl = 0.001;
K0 = cartesian_full_stiffness_tensegrity(p,robot,act_rod,act_node);

K_tens = zeros(24,3*length(act_rod),length(robot.active_springs));
for i = 1:length(robot.active_springs)
    robot.l(robot.active_springs(i))=robot.l(robot.active_springs(i)) + dl;
    K = cartesian_full_stiffness_tensegrity(p,robot,act_rod,act_node);
    robot.l(robot.active_springs(i))=robot.l(robot.active_springs(i)) - dl;
    K_tens(:,:,i) = (K - K0)/dl;
end
end