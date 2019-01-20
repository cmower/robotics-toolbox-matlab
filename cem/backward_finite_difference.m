function fd = backward_finite_difference(fun, x, m, n, h)
%BACKWARD_FINITE_DIFFERENCE Estimates the derivative of a function.
%   fd = BACKWARD_FINITE_DIFFERENCE(fun, x, m, n, h) computes an estimate
%   fd of the derivative of the function fun at x where fun is a m-vector
%   valued function of n-vector input. h is a small increment (default
%   1e-6). Note, there are no checks on the input.
% 
% By C. E. Mower, 2019.
%

% Setup 
if nargin < 4, h = 1e-6; end
I = eye(n);
fd = zeros(m, n);

% Compute fd
for i = 1:n
    fd(:,i) = fun(x) - fun(x - h*I(:,i));
end
fd = fd / h;

end