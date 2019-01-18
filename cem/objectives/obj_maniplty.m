function p = obj_maniplty(robot, q, dof)
%TM_MANIPLTY

if nargin < 3, dof = 1; end

if isscalar(dof)
    if dof == 1
        % position
        dof = [1 1 1 0 0 0];
    elseif dof == 2
        % rotation
        dof = [0 0 0 1 1 1];
    elseif dof == 3
        % all
        dof = [1 1 1 1 1 1];
    else
        error('did not recognise dof');
    end 
end

if length(dof)~=6, error('dof is not correct length, should be 6'); end

m = robot.maniplty(q, 'dof', dof);
p = -m^2;

end