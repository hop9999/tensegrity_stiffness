function K = cartesian_full_stiffness_tensegrity(p,robot,act_rods,act_nodes)
alpha = 1/100;
robot.f = [0;0;0];
%robot = inv_kin_tensegrity(p,robot);
[p0,pos] = forward_kin_tensegrity(robot);
n_base = length(robot.base)/6;
K = zeros(length(pos), 3*length(act_rods));
for i = 1:length(act_rods)
        robot.end_eff.rod = act_rods(i);
        robot.end_eff.end = act_nodes(i);
        robot.f = alpha*[1;0;0];
        [px,posx] = forward_kin_tensegrity(robot);
        robot.f =  alpha*[0;1;0];
        [py,posy] = forward_kin_tensegrity(robot);
        robot.f =  alpha*[0;0;1];
        [pz,posz] = forward_kin_tensegrity(robot);
        K(:,(act_rods(i) - n_base - 1)*6 + (act_nodes(i)-1)*3 + (1:3)) = [(posx - pos)',(posy - pos)',(posz - pos)']/alpha;
end
end