function d = spring_energy(x,a1,a2,b1,b2,robot,index)
    x1 = x((a2-1)*3+1:a2*3,a1);
    x2 = x((b2-1)*3+1:b2*3,b1);
    l_ef = norm(x1-x2)-robot.l(index);
    if l_ef < 0
        d = 0;
    else
        d = robot.k(index)*(norm(x1-x2)-robot.l(index))^2/2;
    end
end