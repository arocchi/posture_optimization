function [ we2_x ] = nlopt_maxFx( q_t )
% direct_RRR_q_maxFx maximizes force along the x direction

n_joint = size(evalin('base','q_ub'),2);
robot_name = evalin('base','robot_name');
robot = evalin('base',robot_name);
q = q_t(1:n_joint);
tau = q_t(n_joint+1:end);

J = robot.jacob0(q');
g = robot.gravload(q');
pJt = pinv(J');
w = pJt*(tau'-g);

%%  
    we2_x = -w(3);
end
