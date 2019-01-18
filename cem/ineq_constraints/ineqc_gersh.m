function c = ineqc_gersh(robot, q)
%INEQC_GERSH 

% Setup
eps_safe = 0;
J = robot.jacobe(q);
J = J(1:3, :); % preserve only position part
A = J*J';
Aabs = abs(A);

c = zeros(3,1);
for i = 1:3
    Aii = A(i,i);
    Ri = sum(Aabs(i,:)) - Aabs(i,i);
    c(i) = eps_safe - Aii + Ri;
end

end

