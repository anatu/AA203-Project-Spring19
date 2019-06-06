function K = robust_MPC(A,B,Q,R,n,MAX_NOISE)
    A1 = A;
    A1(1,1) = A1(1,1) - 1;
    A1(2,2) = A1(2,2) - 1;
    A1(1,2) = A1(1,2) - MAX_NOISE;
    A2 = A;
    A2(1,1) = A2(1,1) - 1;
    A2(2,2) = A2(2,2) - 1;
    A2(1,2) = A2(1,2) + MAX_NOISE;

    Y = sdpvar(n,n);
    L = sdpvar(1,n,'full');

    F = [Y >= 0];
    F = [F, [-A1*Y-B*L + (-A1*Y-B*L)' Y L';Y inv(Q) zeros(n,1);L zeros(1,n) inv(R)] >= 0];
    F = [F, [-A2*Y-B*L + (-A2*Y-B*L)' Y L';Y inv(Q) zeros(n,1);L zeros(1,n) inv(R)] >= 0];
    
    optimize(F,-trace(Y))
    K = value(L)*inv(value(Y));
end