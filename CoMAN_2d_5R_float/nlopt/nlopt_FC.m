function [c1] = nlopt_FC( q_t )
% check frìction cones
    n_joint = size(evalin('base','q_ub'),2);
    robot_name = evalin('base','robot_name');
    robot = evalin('base',robot_name);
%     J = evalin('base','J;');
     q = q_t(1:n_joint);
     tau = q_t(n_joint+1:end);

%    w = evalin('base','w;');

    J = robot.jacob0(q');
    g = robot.gravload(q');
    pJt = pinv(J');
    w = pJt*(tau'-g);

    
    fxe1 = w(1);
    fye1 = w(2);
    fxe2 = w(3);
    fye2 = w(4);
    mu1 = evalin('base','mu1;');
    mu2 = evalin('base','mu2;');
    fc_wall = -mu2*abs(fxe2) + abs(fye2);
    fc_floor = -mu1*abs(fye1) + abs(fxe1);
    % |fye2| <= mu*|fxe2|
    % |fye1| <= mu*|fxe1|
    c1 = max([fc_wall;fc_floor]); %c1 must be <= 0
end
