function cfg=robot_parameters
%ROBOT_PARAMETERS Parameters for the six-rod, three-motor robot.
% Modify cfg before calling CPRRobot(cfg). Lengths are in meters, forces
% in newtons and motor angles in radians; plot labels use mm and degrees.
cfg=struct;
cfg.preset='custom';

% The six rods share this material and discretization.
cfg.N=4;                         % elements per rod
cfg.L=.19;                       % rod length, m
cfg.radius=.5e-3;                % radius, m
cfg.E=1.13e10;                   % Young's modulus, Pa
cfg.nu=.3;
cfg.rho=10;                      % source prototype's density, kg/m^3
cfg.xiNatural=[0;0;0;0;0;1];     % zero natural curvature, local z axis
cfg.platformMass=.025;           % kg

% Geometry can be changed before constructing CPRRobot.
cfg.geometry=cpr.default_geometry;
% Column j defines the fixed world axis of physical motor j.
cfg.axisDirections=[[1;0;0],[-.5;-sqrt(3)/2;0],[-.5;sqrt(3)/2;0]];
cfg.axisPoints=[[0;-.03;-.01],[-.03*sqrt(3)/2;.015;0],[.03*sqrt(3)/2;.015;0]];
cfg.commandForMotor=1:3;         % direct physical numbering in this API
cfg.initialMotorAngles=zeros(3,1);
% Positive motor angles follow the right-hand rule about these axes.

% Loads. They can also be changed at runtime through the object methods.
cfg.gravity=[0;0;-9.81];
cfg.pulley.anchor=[.1432;.075;.150];
cfg.pulley.offset=cfg.geometry.pulleyOffset;
cfg.pulley.tension=0;            % N; set e.g. 0.020*9.81 for a 20 g weight

cfg.dt=.1;                      % command interval, s (quasi-static solves)
cfg.operator='closed';
cfg.samplesPerRod=201;           % shape samples per rod; does not change the solve mesh
cfg.solver.tolerance=1e-8;       % infinity norm of the scaled residual
cfg.solver.maxIterations=180;    % update limit for each continuation attempt
cfg.solver.maxBacktracks=18;
cfg.solver.minStep=2^-18;
cfg.solver.maxScaledStep=.5;
cfg.solver.verbose=false;
cfg.continuation.minFraction=1/128;
cfg.visualization.enabled=false;
cfg.visualization.every=1;
cfg.visualization.showMotorLabels=true;
% Display settings. view = [azimuth, elevation] in degrees.
cfg.visualization.view=[38 24];
cfg.visualization.lockView=true; % fixed camera; disable mouse rotation and zoom
cfg.visualization.axisLimits=[]; % [] for fixed automatic limits, or [xmin xmax;ymin ymax;zmin zmax] in m
% Motor blocks follow the motor poses. These dimensions affect only the drawing.
cfg.visualization.motorWidth=.026;
cfg.visualization.motorHeight=.010;
cfg.visualization.motorEndPadding=.006;
cfg.visualization.platformThickness=.004;
cfg.visualization.rodLineWidth=2.6;
end
