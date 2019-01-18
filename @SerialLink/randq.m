function Q = randq(robot, n)
%RANDQ 

if nargin < 2, n = 1; end

qup = robot.qlim(:,1)';
qdown = robot.qlim(:,2)';

Q = qdown + (qup - qdown).*rand(n,robot.n);

end

