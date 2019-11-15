function P = energy_sphere(q,robot)  
    n_rods = length(q)/6;
    n_base = length(robot.base)/6;
    
    x = zeros(6,n_rods + n_base);
    g = 9.8;

    for i = 1:n_base
        x(1:3,i) = [ robot.base((i-1)*6 + 1)
                     robot.base((i-1)*6 + 2)
                     robot.base((i-1)*6 + 3) ];

        x(4:6,i) = [ robot.base((i-1)*6 + 4)
                     robot.base((i-1)*6 + 5)
                     robot.base((i-1)*6 + 6) ];
    end
    
    P_grav = 0;
    for i = n_base + 1:n_rods + n_base
        x(1:3,i) = [ q((i-1-n_base)*6 + 1)
                     q((i-1-n_base)*6 + 2)
                     q((i-1-n_base)*6 + 3) ];

        x(4:6,i) = [ q((i-1-n_base)*6 + 4)
                     q((i-1-n_base)*6 + 5)
                     q((i-1-n_base)*6 + 6) ];
        P_grav = P_grav + robot.m(i)*g*(q((i-1-n_base)*6 + 3) +  q((i-1-n_base)*6 + 6))/2;
    end
    P_ext = transpose(robot.f)*x(3*(robot.end_eff.end-1)+(1:3),robot.end_eff.rod);
    P_elast = spring_energy(x,1,1,3,1,robot,1) + spring_energy(x,1,1,3,2,robot,2) + spring_energy(x,1,1,5,1,robot,3) + ...
        spring_energy(x,1,1,6,1,robot,4) + spring_energy(x,3,1,6,1,robot,5) + spring_energy(x,3,2,5,1,robot,6) + ...
        spring_energy(x,6,1,1,2,robot,7) + spring_energy(x,5,1,1,2,robot,8) + spring_energy(x,1,2,4,2,robot,9) + ...
        spring_energy(x,1,2,4,1,robot,10) + spring_energy(x,4,2,2,2,robot,11) + spring_energy(x,4,1,2,2,robot,12) + ...
        spring_energy(x,3,2,2,1,robot,13) + spring_energy(x,3,1,2,1,robot,14) + spring_energy(x,2,2,5,2,robot,15) + ...
        spring_energy(x,2,1,5,2,robot,16) + spring_energy(x,2,2,6,2,robot,17) + spring_energy(x,2,1,6,2,robot,18) + ...
        spring_energy(x,6,1,4,1,robot,19) + spring_energy(x,5,1,4,2,robot,20) + spring_energy(x,4,2,5,2,robot,21) + ...
        spring_energy(x,3,2,5,2,robot,22) + spring_energy(x,4,1,6,2,robot,23) + spring_energy(x,3,1,6,2,robot,24);
    P = (P_elast + P_grav + P_ext);

end     