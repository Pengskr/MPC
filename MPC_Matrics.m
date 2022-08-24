function [E, H] = MPC_Matrics(A, B, C, Q, F, R, n, p, N, r, xk)
    M = [eye(n);zeros(N*n, n)];
    D = zeros((N+1)*n, N*p);
    
    tmp = eye(n);
    % 更新M和C
    for i = 1:N
        rows = i*n + (1:n);
        D(rows, :) = [tmp*B, D(rows-n, 1:end-p)];
        tmp = A*tmp;
        M(rows, :) = tmp;
    end
    
%     Q_bar = zeros((N+1)*n);
%     R_bar = zeros(N*p);
    
    Q1_bar = kron(eye(N), C'*Q*C);
    Q1_bar = blkdiag(Q1_bar, C'*F*C);
    Q2_bar = kron(ones(N,1), C'*Q*r);
    Q2_bar = [Q2_bar; C'*F*r];
    R_bar = kron(eye(N), R);
    
    E = D'*Q1_bar*M*xk - D'*Q2_bar;
    H = D'*Q1_bar*D + R_bar;
end