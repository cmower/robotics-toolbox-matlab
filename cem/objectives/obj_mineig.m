function f = obj_mineig(robot, q)
%OBJ_MINEIG 

J = robot.jacobe(q);
A = J*J';
f = -min(eig(A));


end

