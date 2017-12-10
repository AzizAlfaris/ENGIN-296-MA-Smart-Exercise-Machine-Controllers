%% Abdulaziz Alfaris. ENGIN 296MA miniproject 

%% Controller platform. 


%% Simple Plant Model 

% state z = [x_dot;vf]. x_dot is the velocity of the paddle. vf is the
% velocity of the flywheel (or can be thought of as the compressive force of the
% mechanical spring). vf is used to model an energy storage element. 

% Numbers are place holders. 
M= 6;% M is the moment of inertia of the 4 bar linkage (averaged around 2*pi for linearity).
Mf = 10; % moment of inertia of a fictious flywheel OR 1/k where k is the stiffnes of a spring (energy storage element)
C = 5;  % C*x_dot^2 is the Coriolis and centripetal forces. 

Ma = [M 0; 0 Mf]; Ca = [C 0; 0 0];

A = -inv(Ma)*Ca; 
% We have 3 inputs. T, F, and Tf. so U = [T;F;Tf]; 
B= inv(Ma)*[1 1 0; 0 0 1]; 

% Note that this plant is simplified and probably won't be suffecient. More
% work should be done on modeling and linearizing the real plant. However,
% the following control code should be suffecient once the plant model is
% figured out. 


%% First Controller: LQR 

% Infinite horizon LQR controller that gets the plant to 0. (Note: 0 is
% arbitrary equilibrium point. It can be changed with another desired one
% such as the optimal velocity from the Hill Surface. 

% OpenLoop system  
G0 = ss(A,B,eye(2),zeros(2,3));
Q = eye(2); % Needs tuning 
R = eye(3); % Needs tuning 
K = lqr(A,B,Q,R);

% New system (Closed Loop) 
CLSYS = feedback(G0,K); 

step(CLSYS) % gives the step response for every input to every output (6 responses) 

% Note: More complicated inputs can be applied at the same time using lsim 



%% Second Controller: MPC 

% Receding horizon controller. Optimization is done using YALMIP toolbox. 
z0 = rand(2,1); 
zmin = [0;0]; zmax = [100;100]; umin=[0;0;0]; umax= [100;100;100]; % Place holders. 
N = 20; % number of steps to get to the target point 
M= 4; % Horizon 
zOpt = sdpvar(2,N+1); % optimal state trajectory  (N+1) to include initial condition.
uOpt = sdpvar(3,N); 
zOpt(:,1) = z0; % initial condition 
zref = rand(2,1); % desired states (terminal 
%options = sdpsettings('verbose',false,'solver','ipopt'); if the system is
%nonlinear use a nonlinear solver. 
options = sdpsettings('verbose',false,'solver','quadprog');
for i = 1:N 
    cost = 0; % initialize 
    constraints = []; % initialize
    z = sdpvar(2,M+1); % states for the MPC horizon
    u = sdpvar(3,M); % inputs for the MPC horizon
    z0 = zOpt(:,N); % redefine the initial condiotion with each loop. 
    for j = 1:M % Constraint and cost loop 
       constraints = [constraints; zmin <= z(:,M) <= zmax; % min and max limits on the states
           z(:,M+1) == A*z(:,M) + B*u(:,M); % System dynamics
           umin <= u(:,M) <= umax; % min and max limits on the inputs. 
           z(:,1) == z0];
           cost = cost + (z(:,M) - zref)'*Q*(z(:,M) - zref) + u(:,M)'*R*u(:,M);  
    end 
    sol = optimize(constraints,cost,options); 
    zOptMPC = double(z); 
    uOptMPC = double(u);
    % pick the first input and state 
    zOpt(:,N+1) = zOptMPC(:,1); 
    uOpt(:,N) = uOptMPC(:,1);  
end 

zOpt; % Optimal trajectory 
uOpt; % Optimal inputs sequence 

% Note that the MPC will still work even if we have a nonlinear system. We
% just need to have a state estimator and observer to get the actual
% trajectory so that we can feed them as initial conditions. 