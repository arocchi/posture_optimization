% NOTE: this is a temporary solution. We should check for self collisions
% in the future by using OpenRave or PQP
function [c3] = nlopt_GEOM( q_t )
% bounds on position of end effectors
    n_joint = size(evalin('base','q_ub'),2);
    q = q_t(1:n_joint);
    robot_name = evalin('base','robot_name;');
    robot = evalin('base',[robot_name ';']);
    
    fk = robot.fkine(q);
    fk_e1 = fk{1};
    fk_e2 = fk{2};
    fk_j1 = fk{3};
    fk_j2 = fk{4};
    fk_j3 = fk{5};
    fk_j4 = fk{6};
    fk_j5 = fk{7};
    y_e1 = fk_e1(2,3);
    y_e2 = fk_e2(2,3);
    y_j1 = fk_j1(2,3);
    y_j2 = fk_j2(2,3);
    y_j3 = fk_j3(2,3);
    y_j4 = fk_j4(2,3);
    y_j5 = fk_j5(2,3);
    x_e1 = fk_e1(1,3);
    x_e2 = fk_e2(1,3);
    x_j1 = fk_j1(1,3);
    x_j2 = fk_j2(1,3);
    x_j3 = fk_j3(1,3);
    x_j4 = fk_j4(1,3);
    x_j5 = fk_j5(1,3);
    % x_e2 > x_e1
    % y_e2 > y_e1
    % y_e1 at bottom
    % x_e2 at right
    c3 = max([max([x_e1, x_j1, x_j2, x_j3, x_j4, x_j5]);y_e1]-[x_e2;min([y_e2, y_j1, y_j2, y_j3, y_j4])]); % bounds on ee position
end
