clc
clear
close all

% 定义矩阵A B
A = [1  0.1;
     -1   2];
n = size(A, 1);
B = [0.2 1;
     0.5 2];
p = size(B, 2);
% 定义矩阵Q F R
Q = [100 0;
      0  1];
F = [100 0;
      0  1];
R = [1 0;
     0 0.1];
% 定义仿真步数k
k_steps = 100;
% 定义矩阵X_k和U_k，用于保存k步的状态向量和控制向量
X_k = zeros(n, k_steps);
U_k = zeros(p, k_steps);
X_k(:, 1) = [20; -20];  % 初始状态


% 定义预测区间
N = 5;
[E, H] = Copy_of_MPC_Matrics(A, B, Q, F, R, n, p, N);

for k = 1:k_steps
    U_k(:, k) = Copy_of_Prediction(X_k(:, k), E, H, N, p);
    X_k(:, k+1) = A*X_k(:, k) + B*U_k(:, k);
end


subplot(2, 1, 1);
hold on;
for i =1 :size (X_k,1)
    plot (X_k(i,:));
end
legend("x1","x2")
grid on
grid minor
hold off;

subplot(2, 1, 2);
hold on;
for i =1 : size (U_k,1)
    plot (U_k(i,:));
end
grid on
grid minor
hold off

legend("u1","u2")