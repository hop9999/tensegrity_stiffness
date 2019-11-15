function K = stiffness_tensegrity(p,robot)
    alpha = 1/100;
    robot.f = [0;0;0];
    robot = inv_kin_tensegrity(p,robot);
    
    robot.f = alpha*[1;0;0];
    [px,x2] = forward_kin_tensegrity(robot);
    
    robot.f =  alpha*[0;1;0];
    [py,x2] = forward_kin_tensegrity(robot);
    
    robot.f =  alpha*[0;0;1];
    [pz,x2] = forward_kin_tensegrity(robot);
    K = [px-p,py-p,pz-p]/alpha;
end