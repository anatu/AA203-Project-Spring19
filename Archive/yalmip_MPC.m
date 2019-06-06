yalmip('clear')
clear all

% Model data
A = [2.9 -0.73 0.25;4 0 0;0 1 0];
B = [0.25;0;0];
C = [-0.2 0.04 0.07];
% E = [0.0625;0.0625;0.0625];
nx = 3; % Number of states
nu = 1; % Number of inputs

% MPC data
Q = eye(nx);
R = 0.01;
N = 10;

% Initial state
x0 = [0;0;0];

u = sdpvar(repmat(nu,1,N),repmat(1,1,N));

constraints = [];
objective = 0;
x = x0;
for k = 1:N
 y = C*x;
 x = A*x + B*u{k};
%  objective = objective + norm(Q*x,1) + norm(R*u{k},1);
 objective = objective + norm((y-0.5),1) + norm(u{k},1)*0.01;
 constraints = [constraints, -1 <= u{k}<= 1, y <= 0.5];
end

optimize(constraints,objective);
value(u{1})

%%
ops = sdpsettings('verbose',2);
controller = optimizer(constraints,objective,ops,x0,u{1});

x = [0;0;0]
for i = 1:25
 uk = controller{x};
 x = A*x + B*uk;
 y = C*x;
 y_all(:,i) = y;
end