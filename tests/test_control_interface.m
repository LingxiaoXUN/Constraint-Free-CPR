function report=test_control_interface
%TEST_CONTROL_INTERFACE Check motor numbering, callbacks, rollback and saved outputs.
root=fileparts(fileparts(mfilename('fullpath')));addpath(root);
cfg=robot_parameters;
cfg.N=3; cfg.L=.20; cfg.E=1.05e10; cfg.gravity=zeros(3,1);
cfg.pulley.offset=cfg.pulley.offset+[.001;0;0];
cfg.geometry.motorInitial(1,4,2)=cfg.geometry.motorInitial(1,4,2)+.001;
robot=CPRRobot(cfg);
assert(robot.Model.N==3 && robot.Model.ndof==186);
assert(robot.Model.h==cfg.L/cfg.N && isequal(robot.Model.pulleyOffset,cfg.pulley.offset));
assert(isequal(robot.Model.motorInitial,cfg.geometry.motorInitial));

% Each angle must rotate only its physical motor and its two attached bases.
[base0,motor0]=cpr.base_poses(robot.Model,zeros(3,1));
kinematicError=0;
for motor=1:3
    theta=zeros(3,1); theta(motor)=-.12;
    [base,motors]=cpr.base_poses(robot.Model,theta);
    a=cfg.axisDirections(:,motor); point=cfg.axisPoints(:,motor);
    X=cpr.SE3.exp([theta(motor)*a;zeros(3,1)]); R=X(1:3,1:3);
    expected=[R*motor0(1:3,1:3,motor),point+R*(motor0(1:3,4,motor)-point);0 0 0 1];
    kinematicError=max(kinematicError,norm(motors(:,:,motor)-expected,'fro'));
    for other=setdiff(1:3,motor), assert(isequal(motors(:,:,other),motor0(:,:,other))); end
    for rod=1:6
        if cfg.geometry.motorForRod(rod)~=motor, assert(isequal(base(:,:,rod),base0(:,:,rod))); end
    end
end
assert(kinematicError<1e-12);

info=robot.initialize;assert(info.converged);
for motor=1:3
    previous=robot.MotorAngles;
    robot.setMotorAngles(zeros(3,1));robot.setMotorAngle(motor,deg2rad(-5*motor));
    target=zeros(3,1); target(motor)=deg2rad(-5*motor);
    assert(isequal(robot.MotorAngles,previous)); % targets do not mutate state
    info=robot.solve;assert(info.converged && isequal(robot.MotorAngles,target));
end
robot.setMotorAngles(deg2rad([-8;-12;-16]));
info=robot.solve;assert(info.converged);
robot.setPulleyTension(.03);robot.setGravity([0;0;-9.81]);
info=robot.solve;assert(info.converged && robot.Loads.tension==.03);

callbackTimes=[]; callbackSteps=[];
startStep=robot.Step;startTime=robot.Time;
robot.setCallback(@control);
runReport=robot.run(5,.1);
assert(runReport.completed && numel(callbackTimes)==5);
assert(max(abs(callbackTimes-(startTime+(1:5)*.1)))<1e-12);
assert(isequal(callbackSteps,startStep+(1:5)));
assert(norm(robot.MotorAngles-commands(robot.Time))<1e-14);
assert(abs(robot.Loads.tension-(.03+.002*robot.Time))<1e-14);
robot.setCallback([]);

% Failed commands roll back state, angles, time, and record count atomically.
acceptedState=robot.State;acceptedAngles=robot.MotorAngles;acceptedLoads=robot.Loads;
acceptedTime=robot.Time;acceptedStep=robot.Step;recordCount=numel(robot.Records);
robot.setSolverOptions(struct('maxIterations',0));
robot.setMotorAngle(1,robot.MotorAngles(1)-.1);
failed=robot.run(1,.1);
assert(~failed.completed && ~robot.LastInfo.converged);
assert(isequaln(robot.State,acceptedState) && isequal(robot.MotorAngles,acceptedAngles));
assert(isequal(robot.Loads,acceptedLoads) && robot.Time==acceptedTime && robot.Step==acceptedStep);
assert(numel(robot.Records)==recordCount && ~robot.Attempts(end).commandAccepted);
robot.setSolverOptions(struct('maxIterations',cfg.solver.maxIterations));
info=robot.solve;assert(info.converged);

% Invalid input must be rejected without changing targets.
target=robot.TargetMotorAngles;
try
    robot.setMotorAngles([NaN;0;0]);
    error('cpr:TestFailed','Nonfinite command was accepted.');
catch err
    assert(~strcmp(err.identifier,'cpr:TestFailed'));
end
assert(isequal(robot.TargetMotorAngles,target));

% Reuse the figure; the default locked view restores the configured angle.
fig=robot.plot('off');view(fig.CurrentAxes,23,34);
sameFigure=robot.plot('off');assert(fig==sameFigure);
assert(norm(fig.CurrentAxes.View-cfg.visualization.view)<1e-12);close(fig);

out=fullfile(root,'results',['control_test_',char(datetime('now','Format','yyyyMMdd_HHmmss'))]);
robot.saveResults(out);
saved=load(fullfile(out,'states.mat'));
assert(strcmp(saved.run.caseName,'custom') && saved.run.completed);
assert(~isfile(fullfile(out,'phase_samples.csv'))); % no prescribed paper phases
table=readtable(fullfile(out,'trajectory.csv'));
assert(all(table.theta1_rad==table.motor1_rad) && all(table.theta2_rad==table.motor2_rad) && all(table.theta3_rad==table.motor3_rad));
assert(abs(table.time_s(end)-robot.Time)<1e-12);
report=struct('passed',true,'maxMotorKinematicError',kinematicError,...
    'acceptedCommands',robot.Step,'callbackCalls',numel(callbackTimes),...
    'atomicRollbackPassed',true,'outputDirectory',strrep(out,[root filesep],''),'finalResidual',info.residualScaled);
disp(report);

    function control(step,t,sim)
        callbackTimes(end+1)=t;callbackSteps(end+1)=step;
        T=sim.getPlatformPose;assert(all(isfinite(T(:))));
        sim.setMotorAngles(commands(t));
        sim.setPulleyTension(.03+.002*t);
    end
end
function theta=commands(t)
theta=deg2rad([-8-2*t;-12+3*t;-16-t]);
end
