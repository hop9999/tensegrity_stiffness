function [p,x,exitflag] = forward_kin_tensegrity(robot)

%Structure robot
%robot.k - array of spring stiffness 
%robot.d - array of rod lengths
%robot.l - array of spring lengths
%robot.x0 - array of init positions 
%robot.m - array of rod mass
%robot.f - external force

%robot.end_eff.rod - number of end effector rod
%robot.end_eff.end - number of end of end effector rod



%%%%%%%%%%%%%%%%%%TODO error in f_ext in energy!!!!!!!!!!!!!!!!!!!!!!!!


    [x,fval] = fmincon(@(x)robot.energy_f(x,robot),robot.x0,[],[],[],[],[],[],@(x)rod_constr(x,robot),robot.fminc_options);
    n_base = length(robot.base)/6;
    p = [x((robot.end_eff.rod - n_base - 1)*6 + (robot.end_eff.end - 1)*3 + 1)
         x((robot.end_eff.rod - n_base - 1)*6 + (robot.end_eff.end - 1)*3 + 2)
         x((robot.end_eff.rod - n_base - 1)*6 + (robot.end_eff.end - 1)*3 + 3)];

end