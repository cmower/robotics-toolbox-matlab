function q = solver_con(robot, T, q0)
%SOLVER_CON 

objfun = @(x) objective_function(x, robot, T);
nonlcon = @(x) nonlinear_constraints(x, robot);
q = fmincon(objfun, q0(:), [], [], [], [], [], [], nonlcon);
q = q(:)';

end

function cost = objective_function(x, robot, T)

q = x(:)';
cost = obj_effpos(robot, q, T) + obj_effori(robot, q, T);

end

function [c, ceq] = nonlinear_constraints(x, robot)

q = x(:)';
c = [ineqc_gersh(robot, q)];
ceq = [];

end