function robot = RobotSetup()

robot.k = ones(1,24)*500;
robot.d = ones(6,1);
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

% fix this horror
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

end