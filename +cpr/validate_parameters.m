function validate_parameters(cfg)
%VALIDATE_PARAMETERS Validate physical parameters, array sizes and rigid transforms.
validateattributes(cfg.N,{'numeric'},{'real','finite','scalar','integer','>=',2});
for name={'L','radius','E','platformMass'}
    validateattributes(cfg.(name{1}),{'numeric'},{'real','finite','scalar','positive'});
end
validateattributes(cfg.rho,{'numeric'},{'real','finite','scalar','nonnegative'});
validateattributes(cfg.nu,{'numeric'},{'real','finite','scalar','>',-1,'<',.5});
validateattributes(cfg.xiNatural,{'numeric'},{'real','finite','size',[6 1]});
validateattributes(cfg.initialMotorAngles,{'numeric'},{'real','finite','size',[3 1]});
validateattributes(cfg.gravity,{'numeric'},{'real','finite','size',[3 1]});
validateattributes(cfg.axisDirections,{'numeric'},{'real','finite','size',[3 3]});
validateattributes(cfg.axisPoints,{'numeric'},{'real','finite','size',[3 3]});
assert(all(abs(vecnorm(cfg.axisDirections)-1)<1e-10),'cpr:Axis','Motor axes must be unit vectors.');
assert(isequal(cfg.commandForMotor(:)',1:3),'cpr:MotorOrder',...
    'CPRRobot uses physical motor order [1 2 3]. Start with robot_parameters.');
validateattributes(cfg.pulley.anchor,{'numeric'},{'real','finite','size',[3 1]});
validateattributes(cfg.pulley.offset,{'numeric'},{'real','finite','size',[3 1]});
validateattributes(cfg.pulley.tension,{'numeric'},{'real','finite','scalar','nonnegative'});
validateattributes(cfg.dt,{'numeric'},{'real','finite','scalar','positive'});
validateattributes(cfg.solver.tolerance,{'numeric'},{'real','finite','scalar','positive'});
validateattributes(cfg.solver.maxIterations,{'numeric'},{'real','finite','scalar','integer','nonnegative'});
validateattributes(cfg.solver.maxBacktracks,{'numeric'},{'real','finite','scalar','integer','nonnegative'});
validateattributes(cfg.solver.maxScaledStep,{'numeric'},{'real','finite','scalar','positive'});
validateattributes(cfg.solver.minStep,{'numeric'},{'real','finite','scalar','>',0,'<=',1});
validateattributes(cfg.solver.verbose,{'logical','numeric'},{'scalar','binary'});
assert(any(strcmp(cfg.operator,{'closed','series4','series6'})),'cpr:Operator',...
    'Operator must be closed, series4 or series6.');
validateattributes(cfg.continuation.minFraction,{'numeric'},{'real','finite','scalar','>',0,'<=',1});
validateattributes(cfg.samplesPerRod,{'numeric'},{'real','finite','scalar','integer','>=',2});
validateattributes(cfg.visualization.every,{'numeric'},{'real','finite','scalar','integer','positive'});
validateattributes(cfg.visualization.enabled,{'logical','numeric'},{'scalar','binary'});
validateattributes(cfg.visualization.showMotorLabels,{'logical','numeric'},{'scalar','binary'});
v=cpr.visual_options(cfg);
validateattributes(v.view,{'numeric'},{'real','finite','size',[1 2]});
validateattributes(v.lockView,{'logical','numeric'},{'scalar','binary'});
if ~isempty(v.axisLimits)
    validateattributes(v.axisLimits,{'numeric'},{'real','finite','size',[3 2]});
    assert(all(v.axisLimits(:,2)>v.axisLimits(:,1)),'cpr:PlotLimits',...
        'Each axis maximum must exceed its minimum. Plot limits use meters.');
end
for name={'motorWidth','motorHeight','motorEndPadding','platformThickness','rodLineWidth'}
    validateattributes(v.(name{1}),{'numeric'},{'real','finite','scalar','positive'});
end
g=cfg.geometry;
check_frames(g.platformInitial,[4 4]); check_frames(g.motorInitial,[4 4 3]);
check_frames(g.baseLocal,[4 4 6]); check_frames(g.tipLocal,[4 4 6]);
validateattributes(g.motorForRod,{'numeric'},{'real','finite','vector','numel',6,'integer','>=',1,'<=',3});
assert(all(histcounts(g.motorForRod,.5:3.5)==2),'cpr:Topology','This robot has two rods per motor.');
end
function check_frames(T,dims)
validateattributes(T,{'numeric'},{'real','finite','size',dims});
for j=1:size(T,3)
    R=T(1:3,1:3,j);
    assert(norm(R'*R-eye(3),'fro')<1e-8 && abs(det(R)-1)<1e-8 && ...
        norm(T(4,:,j)-[0 0 0 1])<1e-12,'cpr:Frame','Geometry must contain valid SE(3) frames.');
end
end
