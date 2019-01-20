function q = solver_uncon(robot, T, q0)
%SOLVER 

objfun = @(x) objective_function(x, robot, T);
q = fminunc(objfun, q0(:));
q = q(:)';

end

function cost = objective_function(x, robot, T)

q = x(:)';
cost = obj_effpos(robot, q, T) + obj_effori(robot, q, T) + obj_mineig(robot, q);

end
