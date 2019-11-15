function [robot,feasible] = inv_kin_tensegrity(p_task,robot)

    [p,x] = forward_kin_tensegrity(robot);
    err = norm(p_task - p);
    feasible = 1;
    n = 1;
    while err > 0.001
        K = jacobian_tensegrity(robot);
        invK = pinv(K);

        dp = p_task - p;
        dl = transpose(invK * dp);
        for i = 1:length(robot.active_springs)
            robot.l(robot.active_springs(i)) = robot.l(robot.active_springs(i)) + dl(i);
        end
        [p,x] = forward_kin_tensegrity(robot);
        err = norm(p_task - p)
        n = n+1;
        if n == 10
            "false unfeas"
            feasible = -1;
            break
        end
    end
end