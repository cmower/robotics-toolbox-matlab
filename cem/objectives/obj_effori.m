function f = obj_effori(robot, q, T)
%OBJ_EFFORI

r = t2r(robot.fkine(q));
t = t2r(T);

f = norm(r - t)^2;

end

