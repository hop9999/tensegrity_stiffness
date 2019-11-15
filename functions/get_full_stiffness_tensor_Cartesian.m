function H_tensor = get_full_stiffness_tensor_Cartesian(robot, varargin)

Parser = inputParser;
Parser.FunctionName = 'get_full_stiffness_tensor_Cartesian';
Parser.addOptional('alpha', 0.001);
Parser.addOptional('active_springs', [1 2 3]);
Parser.addOptional('act_rods', [1 2 3]);
Parser.addOptional('act_nodes', [1 2 3]);
Parser.parse(varargin{:});

alpha = Parser.Results.alpha;
active_springs = Parser.Results.active_springs;
act_rods = Parser.Results.act_rods;
act_nodes = Parser.Results.act_nodes;

temp.l = robot.l;

% x = forward_kinematics(robot);

K0 = get_full_stiffness_matrix_Cartesian(robot, act_rods, act_nodes);

H_tensor = zeros(24,3*length(act_rod),length(active_springs));
for i = 1:length(active_springs)
    robot.l(active_springs(i))=robot.l(active_springs(i)) + alpha;
    
    K = get_full_stiffness_matrix_Cartesian(robot, act_rods, act_nodes);
    
    robot.l(active_springs(i))=robot.l(active_springs(i)) - alpha;
    
    H_tensor(:,:,i) = (K - K0)/alpha;
end

robot.l = temp.l;
end