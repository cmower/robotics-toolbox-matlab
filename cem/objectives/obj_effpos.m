function f = obj_effpos(robot, q, T)
%EFFPOS_TASK_MAP 

t = transl(T); t = t(:)'; % target
e = transl(robot.fkine(q)); e = e(:)'; % eff pos

f = norm(t - e)^2;

end

