%% generate A, B matrices
clear all; close all;
A = [1 1; 0 1];
B = [0;1];

n = length(B);
N = 4; % MPC horizon
N_sim = 10; % Simulation horizon

%% objective, constraints, initial state

Q = eye(n); R = 0.01; 
xmax = [1;1]; xmin = -xmax;
umax = 1; umin = -umax;

[P,~,~] = idare(A,B,Q,R);
x0 = [-1;1];

%% Setup Gaussian white noise
MAX_NOISE = 0.5;
pd = makedist('Normal');
normal_trunc = truncate(pd,-MAX_NOISE,MAX_NOISE);

%% MPC
[Xallmpc,optvalmpc] = MPC(A,B,P,Q,R,xmax,umax,xmin,umin,x0,normal_trunc,N,N_sim);
[Xallmpc_perturb,optvalmpc_perturb] = MPC_perturb(A,B,P,Q,R,xmax,umax,xmin,umin,x0,normal_trunc,N,N_sim);

%%
% K = robust_MPC(A,B,Q,R,n,MAX_NOISE);
%%
% A_perturb = A;
% Xallmpc = zeros(2,N+1); Uallmpc = zeros(1,N);
% 
% x = x0; %reset initial state
% Xallmpc(:,1) = x;
% for i = 1:N
%     u = K*x;
%     A_perturb(1,2) = A(1,2) + random(normal_trunc);
%     %forward propagate state
%     x = A_perturb*x+B*u;
%     Xallmpc(:,i+1) = x;
%     Uallmpc(:,i) = u;
% end


%% Plots 
plot(Xallmpc(1,:),Xallmpc(2,:));
hold on
% plot(Xallmpc(1,end),Xallmpc(2,end),'o')
plot(Xallmpc_perturb(1,:),Xallmpc_perturb(2,:));
% plot(Xallmpc_perturb(1,end),Xallmpc_perturb(2,end),'o')
xlabel('x_1')
ylabel('x_2')
title('Perfect system MPC vs MPC with unknown (bounded) perturbation initial state x0 = [-4.5,2]')
legend('Perfect System Dynamics','Perturbed System Dynamics')