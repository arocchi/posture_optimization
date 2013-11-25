%MAXIMIZE_FX

cd robot;
genRobots;
cd ..;

%optimize_using = 'Direct';
optimize_using = 'nlopt';
%nlopt_alg = NLOPT_GN_MLSL;
nlopt_alg = NLOPT_GN_ISRES;
%nlopt_alg = NLOPT_LN_COBYLA;
%nlopt_alg = NLOPT_GN_ORIG_DIRECT;
nlopt_alg_loc = NLOPT_LN_COBYLA;

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

mu1 = 1.4;
mu2 = 1.4;

%% chosing optimization method
if exist('optimize_using','var') && strcmp(optimize_using,'nlopt')==1
    cd nlopt;
    opt = struct();
    opt.algorithm = nlopt_alg;
    
    opt.maxeval  = 50000;  %Maximum Number of Function Evaluations
           
    opt.lower_bounds = [q_lb'; tau_lb'];
    opt.upper_bounds = [q_ub'; tau_ub'];
    
    opt.local_optimizer.algorithm = nlopt_alg_loc;
    
    opt.min_objective = @nlopt_maxFx;
    if nlopt_alg == NLOPT_GN_ORIG_DIRECT
        opt.fc = { @nlopt_FC, @nlopt_GEOM, @nlopt_FLATFOOT };
        opt.fc_tol = [1e-4, 1e-4, 1e-3];
    else
        opt.fc = { @nlopt_FC, @nlopt_GEOM };
        opt.fc_tol = [1e-4, 1e-4];
        opt.h = { @nlopt_FLATFOOT };
        opt.h_tol = [1e-3];
    end

    opt.xtol_rel = 1e-4;
    
    tic;
    [q_t,fmin,retcode] = nlopt_optimize(opt, zeros(1,16));
    toc
    switch retcode
        case 6
            display('maxtime reached');
        case 5
            display('maxeval reached');
        case 4
            display('xtol reached');
        case 3
            display('ftol reached');
        case 2
            display('stopval reached');
        case 1
            display('generic success code');
        case -1
            display('generic failure code');
        case -2
            display('invalid arguments');
        case -3
            display('out of memory');
        case -4
            display('roundoff-limited');
        otherwise
            display('unknown error');
    end
    
    q = q_t(1:size(q_ub,2));
    tau = q_t(size(q_ub,2)+1:end);
    
    J = robot.jacob0(q');
    g = robot.gravload(q');
    J = J([1 2 4 5],:); % consider 4-dimensional wrench vector
    pJt = pinv(J');
    w = pJt*(tau'-g);
    
    fk = robot.fkine(q');
    fk_e1 = fk{1};
    fk_e2 = fk{2};
    q([1 2]) = [-fk_e2(1,3) -fk_e1(2,3)];
    robot.plot(q);
    save(['data-' mfilename '_' datestr(now,'yymmdd_HHMMSS') '.mat'],'q','w','tau',robot_name,'q_ub','q_lb','tau_ub','tau_lb','mu1','mu2');
    cd ..;
else
    display('Unsupported optimization method');
end
