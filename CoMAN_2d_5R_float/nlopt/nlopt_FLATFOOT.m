% NOTE: this is a temporary solution. We should check for self collisions
% in the future by using OpenRave or PQP
function [flat_foot] = nlopt_FLATFOOT( q_t )
% bounds on position of end effectors
    n_joint = size(evalin('base','q_ub'),2);
    q = q_t(1:n_joint);
    robot_name = evalin('base','robot_name;');
    robot = evalin('base',[robot_name ';']);
    
    % theta_y + q3 + q4 + q5 = 0
    A = [0 0 1 0 0 1 1 1]; b = 0;
    flat_foot = abs(A*q' - b);
end
