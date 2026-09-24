% Apply motor commands and rope tension, then plot the resulting equilibrium.
root=fileparts(fileparts(mfilename('fullpath')));addpath(root);
cfg=robot_parameters;
robot=CPRRobot(cfg);
info=robot.initialize;assert(info.converged);
robot.setMotorAngles(deg2rad([-10;-5;-15]));
robot.setPulleyTension(.020*9.81);
info=robot.solve;assert(info.converged);
robot.plot;
