n = 2;
A = [1 1; 0 1];
B = [0;1];
C = [1,1];
E = [1;1];

MAX_NOISE = 0.5;
pd = makedist('Normal');
normal_trunc = truncate(pd,-MAX_NOISE,MAX_NOISE);

N = 4;
% N_sim = 10;
Q = eye(n); R = 0.01; 
U = sdpvar(N,1);
W = sdpvar(N,1);
x = sdpvar(n,1);

[P,~,~] = idare(A,B,Q,R);

X = [];
xk = x;
for k = 1:N
 xk = A*xk + B*U(k)+E*W(k);
 X = [X;xk];
end

objective = norm(X,2) + norm(U,2)*R;
% objective = objective + norm(P*xk,2) - norm(Q*xk,2) - norm(R*U(k),2);
F = [ -1 <= U <= 1];
% objective = norm(X-1,1) + norm(U,1)*0.01;
G = [-MAX_NOISE <= W <= MAX_NOISE];
[Frobust,h] = robustify(F + G,objective,[],W);

%%
xk = [-1;1];
ops = sdpsettings;
for i = 1:N_sim
    optimize([Frobust, x == xk(:,end)],h,ops);
    xk = [xk A*xk(:,end) + B*value(U(1)) + E*random(normal_trunc)];
end
hold on
plot(xk(1,:),xk(2,:))
