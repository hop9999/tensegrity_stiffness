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

function [x, fval, exitflag, output] = forward_kinematics(robot, varargin)

Parser = inputParser;
Parser.FunctionName = 'forward_kinematics';
Parser.addOptional('fmincon_options', []);
Parser.parse(varargin{:});

fmincon_options = Parser.Results.fmincon_options;

    [x, fval, exitflag, output] = fmincon(@(x)robot.energy_f(x,robot), robot.x0,[],[],[],[],[],[], @(x)rod_constr(x,robot), fmincon_options);

end