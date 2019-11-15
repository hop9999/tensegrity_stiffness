function K = jacobian_tensegrity(robot)
    dl = 0.0001;
    p0 = forward_kin_tensegrity(robot);

    K = zeros(3,length(robot.active_springs));
    for i = 1:length(robot.active_springs)
        robot.l(robot.active_springs(i))=robot.l(robot.active_springs(i)) + dl;
        p = forward_kin_tensegrity(robot);
        robot.l(robot.active_springs(i))=robot.l(robot.active_springs(i)) - dl;
        dp = (p - p0)/dl;
        K(:,i) = dp;
    end
end