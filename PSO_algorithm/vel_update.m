function v_new = vel_update(v,x,w,pbest,gbest, c)

%Inputs:
%   v - current velocity
%   x - current position
%   w - weight coefficient
%   pbest - personal best position
%   gbest - global best position
%   c - self-learning coefficients
%Outputs:
%   v_new - updated velocity

r = rand([1,1]); % random variable [from 0 to 1]
r2 = rand([1,1]);

v_new = zeros(2,1);
v_new(1) = w.*v(1) + c(1).*r.*(pbest(1)-x(1)) + c(2).*r2.*(gbest(1)-x(1));
v_new(2) = w.*v(2) + c(2).*r.*(pbest(2)-x(2)) + c(2).*r2.*(gbest(2)-x(2));
end

