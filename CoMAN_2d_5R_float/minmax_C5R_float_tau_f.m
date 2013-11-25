%MAXIMIZE_FX

cd robot;
genRobots;
cd ..;

optimize_using = 'Direct';

robot = evalin('base',robot_name);
%% setting algorithm parameters
%q_ub =   [2*pi/3    pi/4    pi/4];
%q_lb =   [  pi/3 -pi/1.1 -pi/1.1];
% NOTE: not having q limits rises conditioning problems which definitely
% influence the choice of the optimal solution among all the global optima
q_ub = [zeros(1,2)  pi  1.66  0     0.79  1.92  0.87];
q_lb = [zeros(1,2) -pi -3.4  -2.36 -1.92 -0.17 -1.22];
%tau_ub = [ 3  2  1];
%tau_lb = [-3 -2 -1];
tau_ub = [0 0 0 30 30 30 30 30];
tau_lb = -tau_ub;
tau_b = tau_ub;

fe2x_d = 10;

f_ub =   [ ones(1,3)*1000 ];
f_lb =   -f_ub;

fi = [0 0];

mu1 = 3;
mu2 = 5;

%% chosing optimization method
if exist('optimize_using','var') && strcmp(optimize_using,'Direct')==1
    cd Direct;
    opts = struct();
    opts.maxevals  = 25000;  %Maximum Number of Function Evaluations
    opts.maxits    = 500;   %Maximum Number of Iterations
    opts.maxdeep   = 100;    %Maximum Number of Rectangle Divisions
    opts.testflag  = 0;     %Global Min known
    opts.globalmin = -2305000;
           
    bounds = [[q_lb'; f_lb'] [q_ub'; f_ub']];
    Problem.f='direct_minMax';

    % forces should be inside friction cones
    Problem.constraint(1).func = 'direct_C1tau_maxFx_f';
    Problem.constraint(1).penalty = 1;    
    % torques should be bounded
    Problem.constraint(2).func = 'direct_C2tau_maxFx_f';
    Problem.constraint(2).penalty = 1;
    % ee1 and ee2 should be in contact with floor and wall
    Problem.constraint(3).func = 'direct_C3tau_maxFx_f';
    Problem.constraint(3).penalty = 1;
    
    Problem.numconstraints=3;
    [fmin,q_f,history] = Direct(Problem, bounds, opts);
    
    q = q_f(1:size(q_ub,2));
    f = q_f(size(q_ub,2)+1:end);
    
    J = robot.jacob0(q');
    J = J([1 2 4 5],:); % consider 4-dimensional wrench vector
    Jt = J';
    n_joint = size(evalin('base','q_ub'),2);
    fe1x = q_f(n_joint+1);
    fe1y = q_f(n_joint+2);
    fe2y = q_f(n_joint+3);
    Fd = [fe1x fe1y fe2x_d fe2y]';
    tau = Jt*Fd;
%     %% displayng stuff (multiple solutions)
%     display('Maximum external applicable force is '); ws(end)
%     
%     if strcmp(using,'RT') == 1
%         h=figure();
%         hold on;
%         for iplot = 1:size(qlocal,2)
%             robot_temp = SerialLink(robot);
%             robot_temp.name = sprintf('%s_%d',robot_name.iplot);
%             robot_temp.plot(qlocal(:,iplot)');
%         end
%         xlabel('X [m]');
%         ylabel('Y [m]');
%         title('Configuration Planning for Maximum Force along X (world coordinates)');
%         set(h,'PaperPositionMode','auto');
%         %set(h,'CameraUpVector',[0 1 0]);    
%         print(h,'-depsc','data.eps');
%     end

    save(['data-' mfilename '.mat'],'q','f','tau',robot_name,'q_ub','q_lb','tau_ub','tau_lb','mu1','mu2');
    cd ..;
else
    display('Unsupported optimization method');
end
