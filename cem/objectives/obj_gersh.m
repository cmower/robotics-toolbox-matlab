function f = obj_gersh(robot, q)
%OBJ_GERSH 

J = robot.jacobe(q);
J = J(1:3,:);
A = J*J';
e = ones(3,1);
R = abs(A) * e - diag(A);

f = -sum(diag(A)) + norm(R - 0.01*e)^2;

end

