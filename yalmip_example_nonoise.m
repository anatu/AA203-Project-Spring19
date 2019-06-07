A = [2.9 -0.73 0.25;4 0 0;0 1 0];
B = [0.25;0;0];
C = [-0.2 0.04 0.07];
E = [0.0625;0.0625;0.0625];

N = 10;
U = sdpvar(N,1);
W = sdpvar(N,1);
x = sdpvar(3,1);

Y = [];
xk = x;
for k = 1:N
 xk = A*xk + B*U(k);
 Y = [Y;C*xk];
end

F = [Y <= 0.5, -1 <= U <= 1];
objective = norm(Y-0.5,1) + norm(U,1)*0.01;

% [Frobust,h] = robustify(F,objective);
xk = [0;0;0];
ops = sdpsettings;
cost = 0;
for i = 1:20
    optimize([F, x == xk(:,end)],objective,ops);
    xk = [xk A*xk(:,end) + B*value(U(1))]
%     xk = [xk A*xk(:,end) + B*value(U(1)) + E*(-1+2*rand(1))];
    cost = cost + value(objective)
end
hold on
plot(C*xk)