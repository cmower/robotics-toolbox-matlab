%% Example 1
%
% Generates random joint configurations for ur10 and plots Gershgorin disks
% for J*J'.
%
% Requirements
%  * matrixcomp
%

clear all;
close all;

ur10 = mdl_ur10;

dof=[1 1 1 0 0 0]; % pos
%dof=[0 0 0 1 1 1]; % rot
%dof=[1 1 1 1 1 1]; % all

dof = logical(dof);

while 1
    
    subplot(1,2,1);
    q = ur10.randq();
    ur10.plot(q);
    axis square;
    
    subplot(1,2,2);
    J = ur10.jacob0(q);
    J = J(dof, :);
    gersh(J*J');
    
    axis square;
    
    pause
    
end

%% Iterating IK (square) example

clear all;
close all;

% ur10 = mdl_ur10;
% robot = ur10;
% q0 = [1.1435198162675,-2.09018404081165,-2.02827206874487,0.976863456308854,0.427276510529307,-3.00484453244653e-10];

jaco = mdl_jaco;
robot = jaco;
q0 = zeros(1, robot.n);

n = 100;



xpos = [linspace(0.5,-0.5,25) -0.5*ones(1, 25) linspace(-0.5, 0.5, 25) 0.5*ones(1,25)];
ypos = [0.5*ones(1, 25) linspace(0.5, -0.5, 25) -0.5*ones(1, 25) linspace(-0.5, 0.5, 25)];
zpos = 0.3*ones(1, 100);

T = @(i) [...
    0   0   1  xpos(i); ...
    -1  0   0  ypos(i); ...
    0   -1  0  zpos(i); ...
    0   0   0  1];

q = q0;
qs = zeros(1, robot.n, n);

%% Iterating IK (sine wave) example

clear all;
close all;

ur10 = mdl_ur10;
robot = ur10;

n = 100;
focus_point.radius=0.05;
focus_point.path.x=ones(1,n);
focus_point.path.y=linspace(0.5, -0.5, n);
focus_point.path.z=0.6 + 0.1*sin(4*pi*focus_point.path.y);
q0 = [0.8478   -2.0187   -1.5015    0.3970    0.7303    0];

q = q0;

qs = zeros(1, robot.n, n);

T = @(i) [...
    0    0    1    0.75;...
    -1   0    0    focus_point.path.y(i);...
    0    -1   0    focus_point.path.z(i);...
    0    0    0    1 ...
    ];

%% Iterating IK solver (ikine, rvctools)

for i = 1:n
    fprintf('%d/%d\n', i, n);
    q = robot.ikine(T(i), 'q0', q);
    qs(:,:,i) = q;
end

%% Iterating IK solver (uncon)

for i = 1:n
    fprintf('%d/%d\n', i, n);
    q = solver_uncon(robot, T(i), q);
    qs(:,:,i) = q;
end

%% Iterating IK solver (con)

for i = 1:n
    fprintf('%d/%d\n', i, n);
    q = solver_con(robot, T(i), q);
    qs(:,:,i) = q;
end

%% Plot iterating IK trajectory

% Setup

plot_figure_1 = true;
plot_target_frames = true;

plot_figure_2 = false;


plot_figure_3 = false;

% Plot
if plot_figure_1
figure(1);
for i = 1:n
    
    subplot(1,2,1);
    fprintf('%d/%d\n', i, n);
    q = qs(:,:,i);
    robot.plot(q);
    drawnow;
    
    hold on;
    
    zlim([-0.1, 1.5]);
    xlim([-1 1]);
    ylim([-1,1]);
    
    if plot_target_frames
        %trplot(T(i));
        targ = transl(T(i));
        plot3(targ(1), targ(2), targ(3), 'xb');
    end
    
    axis square;
    
    subplot(1,2,2);
    J = robot.jacobe(q);
    J = J(1:3,:);
    A = J*J';
    gersh(A);
    
    axis square;
    
end
end

if plot_figure_2
figure(2);
es = zeros(1, n);
for i = 1:n
    q = qs(:,:,i);
    es(i) = obj_effpos(robot, q, T(i));
end
plot(1:n, es);
end

if plot_figure_3
figure(3);
for i = 1:n
    fprintf('%d/%d\n', i, n);
    q = qs(:,:,i);
    J = robot.jacobe(q);
    J = J(1:3,:);
    A = J*J';
    gersh(A);
    pause;
end
end