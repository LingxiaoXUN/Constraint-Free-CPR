classdef CPRRobot < handle
%CPRROBOT Quasi-static control of a six-rod parallel robot.
% Angles are absolute, in radians, in physical motor order [1;2;3].
% Call initialize, set motor or load targets, then call solve or run.
% A controller(step,targetTime,robot) callback may set targets before each
% step. State, MotorAngles and Loads change only when a full step succeeds.

properties (SetAccess=private)
    Config
    Model
    State = []
    MotorAngles = zeros(3,1)       % last accepted angles
    TargetMotorAngles = zeros(3,1)
    Loads
    TargetLoads
    Initialized = false
    Time = 0                     % time of the last accepted command, s
    Step = 0                     % number of accepted commands
    Records = struct([])
    Attempts = struct([])
    LastInfo = struct([])
    InitializationInfo = struct([]) % cold geometric solve, before loading
    Figure = []
end
properties (Access=private)
    Callback = []
end

methods
    function obj=CPRRobot(cfg)
        % Build the model from a configuration returned by robot_parameters.
        if nargin<1, cfg=robot_parameters; end
        cpr.validate_parameters(cfg);
        obj.Config=cfg;
        obj.Model=cpr.model(cfg);
        obj.TargetMotorAngles=cfg.initialMotorAngles;
        obj.Loads=struct('gravity',zeros(3,1),'tension',0);
        obj.TargetLoads=struct('gravity',cfg.gravity,'tension',cfg.pulley.tension);
    end

    function setMotorAngles(obj,theta)
        % Set absolute target angles [motor1; motor2; motor3], in radians.
        % Call solve or run to compute the corresponding equilibrium.
        validateattributes(theta,{'numeric'},{'real','finite','vector','numel',3});
        obj.TargetMotorAngles=theta(:);
    end

    function setMotorAngle(obj,motorId,angle)
        % Set one motor's absolute target angle in radians.
        % The other two target angles are retained.
        validateattributes(motorId,{'numeric'},{'scalar','integer','>=',1,'<=',3});
        validateattributes(angle,{'numeric'},{'real','finite','scalar'});
        obj.TargetMotorAngles(motorId)=angle;
    end

    function setPulleyTension(obj,tension)
        % Set the target rope tension in newtons; zero removes the traction.
        validateattributes(tension,{'numeric'},{'real','finite','scalar','nonnegative'});
        obj.TargetLoads.tension=tension;
    end

    function setGravity(obj,gravity)
        % Set the target world-frame gravity vector [gx;gy;gz], in m/s^2.
        validateattributes(gravity,{'numeric'},{'real','finite','vector','numel',3});
        obj.TargetLoads.gravity=gravity(:);
    end

    function setSolverOptions(obj,options)
        % Runtime solver settings; geometry/material changes need a new robot.
        assert(isstruct(options) && isscalar(options),'cpr:Options','Options must be a scalar struct.');
        candidate=obj.Config; names=fieldnames(options);
        for k=1:numel(names)
            name=names{k};
            assert(isfield(candidate.solver,name),'cpr:Options','Unknown solver option: %s',name);
            candidate.solver.(name)=options.(name);
        end
        cpr.validate_parameters(candidate);
        obj.Config=candidate;
        obj.Model.cfg.solver=candidate.solver;
    end

    function setCallback(obj,callback)
        % Register controller(step,targetTime,robot), or [] to disable it.
        assert(isempty(callback) || isa(callback,'function_handle'),'cpr:Callback',...
            'Use a function handle @(step,t,robot), or [] to disable.');
        obj.Callback=callback;
    end

    function info=initialize(obj)
        % Solve from a geometric guess that satisfies the attachments.
        % First find the unloaded equilibrium at initialMotorAngles, then
        % continue gravity and tension to their requested initial values.
        assert(~obj.Initialized,'cpr:Initialized','Already initialized. Construct a new robot to restart.');
        theta=obj.Config.initialMotorAngles;
        zero=struct('gravity',zeros(3,1),'tension',0);
        guess=cpr.initial_state(obj.Model,theta);
        [state,cold]=cpr.solve(obj.Model,guess,theta,zero);
        obj.InitializationInfo=cold;
        if ~cold.converged
            obj.LastInfo=cold;
            obj.appendAttempts(struct('fromFraction',0,'targetFraction',0,'theta',theta,'load',zero,'info',cold),...
                'initialization',0,0,false);
            info=cold;
            return
        end
        [state,attempts]=cpr.advance(obj.Model,state,theta,zero,theta,obj.TargetLoads);
        success=attempts(end).info.converged && attempts(end).targetFraction==1;
        obj.appendAttempts(attempts,'initial_load',0,0,success);
        info=obj.combineInfo(attempts,success);
        obj.LastInfo=info;
        if success
            obj.State=state;
            obj.MotorAngles=theta;
            obj.Loads=obj.TargetLoads;
            obj.Initialized=true;
            obj.appendRecord(info);
        end
    end

    function info=solve(obj)
        % Solve targets without advancing the user-defined time variable.
        if ~obj.Initialized
            info=obj.initialize;
            if ~info.converged, return; end
        end
        info=obj.advanceAndRecord(obj.Time,obj.Step+1);
    end

    function report=run(obj,nSteps,dt)
        % Advance nSteps commands, calling the controller before each solve.
        % dt specifies command spacing in seconds; the model is quasi-static.
        % Stop at the first failed command and report the accepted step count.
        if nargin<3, dt=obj.Config.dt; end
        validateattributes(nSteps,{'numeric'},{'real','finite','scalar','integer','nonnegative'});
        validateattributes(dt,{'numeric'},{'real','finite','scalar','positive'});
        report=struct('completed',false,'acceptedSteps',0,'requestedSteps',nSteps,'lastInfo',struct([]));
        if ~obj.Initialized
            info=obj.initialize;
            report.lastInfo=info;
            if ~info.converged, return; end
        end
        for k=1:nSteps
            % Advance the accepted time and state only after this target succeeds.
            targetTime=obj.Time+dt;
            targetStep=obj.Step+1;
            if ~isempty(obj.Callback), obj.Callback(targetStep,targetTime,obj); end
            info=obj.advanceAndRecord(targetTime,targetStep);
            report.lastInfo=info;
            if ~info.converged, return; end
            report.acceptedSteps=report.acceptedSteps+1;
            if obj.Config.visualization.enabled && mod(obj.Step,obj.Config.visualization.every)==0
                obj.plot;
                drawnow;
            end
        end
        report.completed=true;
    end

    function T=getPlatformPose(obj)
        % Return the accepted 4-by-4 body-to-world pose; translation is in m.
        assert(obj.Initialized,'cpr:NotInitialized','Initialize the robot first.');
        T=obj.State.platform;
    end

    function fig=plot(obj,visible)
        % Draw the accepted equilibrium, reusing the figure when possible.
        if nargin<2, visible='on'; end
        assert(obj.Initialized,'cpr:NotInitialized','Initialize the robot first.');
        fig=cpr.plot_shape(obj.Model,obj.State,obj.MotorAngles,obj.Loads,visible,obj.Figure);
        obj.Figure=fig;
    end

    function data=results(obj)
        % Export states, parameters and solver histories as plain structures.
        data=struct('caseName','custom','parameterName','time_s','cfg',obj.Config,...
            'initialization',obj.InitializationInfo,'records',obj.Records,'attempts',obj.Attempts,...
            'completed',obj.Initialized && ~isempty(obj.LastInfo) && obj.LastInfo.converged,...
            'environment',struct('matlab',version,'computer',computer,'timestamp',char(datetime('now'))));
        if isempty(obj.Callback), data.controller='manual'; else, data.controller=func2str(obj.Callback); end
        data.targetMotorAngles=obj.TargetMotorAngles;
        data.targetLoads=obj.TargetLoads;
    end

    function saveResults(obj,out)
        % Save MAT/CSV data and FIG/PNG graphics to a new output directory.
        assert(~isfile(fullfile(out,'states.mat')),'cpr:OutputExists','Choose a new output directory.');
        cpr.save_run(out,obj.results);
        if obj.Initialized
            fig=cpr.plot_shape(obj.Model,obj.State,obj.MotorAngles,obj.Loads,'off');
            savefig(fig,fullfile(out,'robot_shape.fig'));
            exportgraphics(fig,fullfile(out,'robot_shape.png'),'Resolution',180);
            close(fig);
        end
    end
end

methods (Access=private)
    function info=advanceAndRecord(obj,targetTime,targetStep)
        [candidate,attempts]=cpr.advance(obj.Model,obj.State,obj.MotorAngles,obj.Loads,...
            obj.TargetMotorAngles,obj.TargetLoads);
        success=attempts(end).info.converged && attempts(end).targetFraction==1;
        obj.appendAttempts(attempts,'command',targetTime,targetStep,success);
        info=obj.combineInfo(attempts,success);
        obj.LastInfo=info;
        if success
            obj.State=candidate;
            obj.MotorAngles=obj.TargetMotorAngles;
            obj.Loads=obj.TargetLoads;
            obj.Time=targetTime;
            obj.Step=targetStep;
            obj.appendRecord(info);
        end
        % A failed command leaves the accepted state unchanged, even when
        % some intermediate substeps converged. Attempts retains their logs.
    end

    function appendRecord(obj,info)
        rec=struct('step',obj.Step,'parameter',obj.Time,'theta',obj.MotorAngles,...
            'state',obj.State,'load',obj.Loads,'info',info);
        obj.Records=[obj.Records,rec];
    end

    function appendAttempts(obj,attempts,stage,time,step,accepted)
        for k=1:numel(attempts)
            attempts(k).parameter=time;
            attempts(k).step=step;
            attempts(k).stage=stage;
            attempts(k).commandAccepted=accepted;
        end
        obj.Attempts=[obj.Attempts,attempts];
    end
end

methods (Static,Access=private)
    function info=combineInfo(attempts,success)
        % Sum work over all attempts, including failed subdivisions.
        allInfo=[attempts.info];
        info=allInfo(end);
        info.converged=success;
        info.updates=sum([allInfo.updates]);
        info.evaluations=sum([allInfo.evaluations]);
        info.seconds=sum([allInfo.seconds]);
        info.loadingAttempts=numel(attempts);
        % Detailed histories, including failed substeps, are in Attempts.
    end
end
end
