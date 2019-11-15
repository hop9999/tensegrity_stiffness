%finds position of the end effector
function end_effector = end_effector_position_from_state(robot, x)

n_base = length(robot.base)/6;
end_effector = [x((robot.end_eff.rod - n_base - 1)*6 + (robot.end_eff.end - 1)*3 + 1)
                x((robot.end_eff.rod - n_base - 1)*6 + (robot.end_eff.end - 1)*3 + 2)
                x((robot.end_eff.rod - n_base - 1)*6 + (robot.end_eff.end - 1)*3 + 3)];
end