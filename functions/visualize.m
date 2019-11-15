function [] = visualize(p,x,robot)
    hold on
    xlim([-1 1])
    ylim([-1 1])
    zlim([-1 1])
    campos('manual')
    campos([-1,-1,1])
    plot3(p(1),p(2),p(3),'r*')
    xlabel("x");
    ylabel("y");
    zlabel("z");
    n_base = length(robot.base)/6;
    axis equal
    colors = {[0 0.4470 0.7410],[0.9900 0.60 0.20], [0.4940 0.1840 0.5560], [0.6350 0.2780 0.1840], [0.3010 0.8450 0.2330]};
    for i = 1:n_base
        plot3([robot.base((i - 1)*6 + 1),robot.base((i - 1)*6 + 4)],...
              [robot.base((i - 1)*6 + 2),robot.base((i - 1)*6 + 5)],...
              [robot.base((i - 1)*6 + 3),robot.base((i - 1)*6 + 6)], 'LineWidth',2,'Color', colors{1});
    end

    for i = 1:length(x)/6
        plot3([x((i-1)*6 + 1),x((i-1)*6 + 4)],...
              [x((i-1)*6 + 2),x((i-1)*6 + 5)],...
              [x((i-1)*6 + 3),x((i-1)*6 + 6)],'Color', colors{1+i});
    end
    
end