function [K, X, poles] = lqr_base(A, B, Q, R)
%LQR_BASE  Continuous-time LQR using only base MATLAB (no Control System Toolbox).
%   Solves A'X + XA - XBR^-1B'X + Q = 0 through the stable eigenspace of the
%   Hamiltonian matrix, then K = R^-1 B' X.  Same answer as lqr(A,B,Q,R).
n = size(A,1);
H = [A, -B/R*B'; -Q, -A'];
[V, D] = eig(H);
stable = real(diag(D)) < 0;
assert(nnz(stable) == n, 'Hamiltonian has eigenvalues on the imaginary axis');
U = V(:, stable);
X = real(U(n+1:end, :) / U(1:n, :));
K = R \ (B' * X);
poles = eig(A - B*K);
end
