function robot=demo_robot_controls(showPlots)
%DEMO_ROBOT_CONTROLS Run direct motor commands followed by callback control.
% demo_robot_controls(false) runs without opening a window.
if nargin<1, showPlots=true; end
root=fileparts(mfilename('fullpath'));addpath(root);

%% Parameters
% Set the geometry, material, loads and display options before construction.
cfg=robot_parameters;
cfg.N=4;
cfg.L=.19;
cfg.E=1.13e10;
cfg.gravity=[0;0;-9.81];
cfg.pulley.tension=0;
cfg.visualization.enabled=showPlots;
cfg.visualization.every=1;
cfg.visualization.view=[38 24];   % initial view: [azimuth, elevation], deg
cfg.visualization.lockView=false;  % false allows mouse rotation and zoom
cfg.samplesPerRod = 20;      % display samples per rod
% cfg.visualization.axisLimits=[-.16 .16;-.16 .16;-.04 .23]; % optional fixed axis limits, m
robot=CPRRobot(cfg);
info=robot.initialize; 
robot.setSolverOptions(struct( ...
    'tolerance', 1e-5, ...      % scaled-residual tolerance
    'maxIterations', 20));     % maximum updates per continuation attempt
assert(info.converged,'Initial equilibrium failed.');
if showPlots, robot.plot; drawnow; end % show the initial equilibrium

%% Direct control: absolute angles of physical motors 1, 2 and 3
robot.setMotorAngles(deg2rad([-10;-20;-5]));
info=robot.solve; assert(info.converged,'Motor command failed.');

% Change motor 2 only. Motors 1 and 3 keep their target angles.
robot.setMotorAngle(2,deg2rad(-25));
info=robot.solve; assert(info.converged,'Single-motor command failed.');

%% Time-dependent motor commands
robot.setCallback(@motor_control);
report=robot.run(30,.1);
assert(report.completed,'A command failed; inspect robot.Attempts.');

out=fullfile(root,'results',['custom_control_',char(datetime('now','Format','yyyyMMdd_HHmmss'))]);
robot.saveResults(out);
fprintf('Saved custom-control run: %s\n',out);
end

function motor_control(step,t,robot) %#ok<INUSD>
% Set absolute motor angles at target time t (seconds).
% The solver runs after this callback. getPlatformPose returns the preceding
% accepted equilibrium and can be used to compute a feedback command.
theta1=deg2rad(-10-15*min(t/2,1));
theta2=deg2rad(-25+15*min(max(t-1,0)/2,1));
theta3=deg2rad(-5-10*sin(pi/2*min(t/3,1)));
robot.setMotorAngles([theta1;theta2;theta3]);
% Additional commands available in this callback:
% robot.setMotorAngle(1,deg2rad(-20));
% robot.setPulleyTension(0.020*9.81);
% robot.setGravity([0;0;-9.81]);
% T = robot.getPlatformPose; % preceding accepted equilibrium for feedback
end
