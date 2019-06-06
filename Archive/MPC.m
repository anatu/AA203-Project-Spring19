function [Xallmpc,optvalmpc] = MPC(A,B,P,Q,R,xmax,umax,xmin,umin,x0,normal_trunc,N,N_sim)
tvec = 0:N_sim;
% T_list = 4;
% N_T = length(T_list);
Qhalf = sqrtm(Q); Rhalf = sqrtm(R); 

% model predictive control w/ different horizons (T) from before
optvalmpc = 0; 

%store solutions
Xallmpc = zeros(2,N_sim+1); Uallmpc = zeros(1,N_sim);

    x = x0; %reset initial state
    Xallmpc(:,1) = x;
    
    fprintf('N=%d ; t = ',N);
    
    %step through time
    for i = 1:N_sim
        fprintf('%d, ',i-1);
        
        %cvx precision
        cvx_precision(max(min(abs(x))/10,1e-6))
        
        cvx_begin quiet
            variables X(2,N+1) U(1,N)
            max(X') <= xmax'; max(U') <= umax';
            min(X') >= xmin'; min(U') >= umin';
            X(:,2:N+1) == A*X(:,1:N)+B*U;
            X(:,1) == x; %initial state constraint
%             X(:,T+1) == 0; %terminal state constraint
            minimize (X(:,N+1)'*P*X(:,N+1)+norm([Qhalf*X(:,1:N); Rhalf*U],'fro'))
        cvx_end
        
        %check feasibility
        if strcmp(cvx_status,'Solved')
            
            %store control
            u = U(:,1);
            Uallmpc(:,i) = u;
            
            %accumulate cost
            if i ~= N_sim
                optvalmpc = optvalmpc + x'*Q*x + u'*R*u;
            elseif i == N_sim
                optvalmpc = optvalmpc + x'*P*x + x'*Q*x + u'*R*u;
            end
            
            %forward propagate state
            x = A*x+B*u;
            
            %record state
            Xallmpc(:,i+1) = x;
            
        else
           % break from loop
           optvalmpc = Inf;
           break;
        end
    end
    fprintf('\n');
end
