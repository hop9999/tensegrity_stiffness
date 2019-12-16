function K = get_full_stiffness_matrix_Cartesian(robot, varargin)

Parser = inputParser;
Parser.FunctionName = 'get_full_stiffness_matrix_Cartesian';
Parser.addOptional('alpha', 0.01);
Parser.addOptional('act_rods', [1 2 3]);
Parser.addOptional('act_nodes', [1 2 3]);
Parser.addOptional('fmincon_options', optimoptions('fmincon', 'Display', 'notify'));
Parser.parse(varargin{:});

alpha = Parser.Results.alpha;
act_nodes = Parser.Results.act_nodes;
act_rods  = Parser.Results.act_rods;
fmincon_options  = Parser.Results.fmincon_options;


temp.f = robot.f;
temp.end_eff = robot.end_eff;

%robot = inv_kin_tensegrity(p,robot);
robot.f = [0; 0; 0];
state = forward_kinematics(robot, 'fmincon_options', fmincon_options);
p = end_effector_position_from_state(robot, state);

n_base = length(robot.base)/6;
K = zeros(length(state), 3*length(act_rods));

for i = 1:length(act_rods)
    
        robot.end_eff.rod = act_rods(i);
        robot.end_eff.end = act_nodes(i);
        
        robot.f = alpha*[1; 0; 0];
        state_x = forward_kinematics(robot, 'fmincon_options', fmincon_options);
%         p_x = end_effector_position_from_state(robot, state_x);
        
        robot.f =  alpha*[0;1;0];
        state_y = forward_kinematics(robot, 'fmincon_options', fmincon_options);
%         p_y = end_effector_position_from_state(robot, state_y);
        
        robot.f =  alpha*[0;0;1];
        state_z = forward_kinematics(robot, 'fmincon_options', fmincon_options);
%         p_z = end_effector_position_from_state(robot, state_z);
        
        K(:,(i-1)*3 + (1:3)) = [(state_x - state)',(state_y - state)',(state_z - state)']/alpha;
end

robot.f = temp.f;
robot.end_eff = temp.end_eff;
end