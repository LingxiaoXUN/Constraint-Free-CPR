function report=test_core
%TEST_CORE Check the energy gradient, pulley tangent and inverse Jacobian.
root=fileparts(fileparts(mfilename('fullpath'))); addpath(root); rng(17);
cfg=robot_parameters;
cfg.L=.22;cfg.E=1e10;cfg.rho=1000;cfg.platformMass=.020;
cfg.gravity=zeros(3,1);cfg.axisPoints(3,1)=0;
m=cpr.model(cfg); state=cpr.initial_state(m,zeros(3,1));
load=struct('gravity',[0;0;-9.81],'tension',.020*9.81);
[r,~,~]=cpr.assemble(m,state,zeros(3,1),load);
epsFD=1e-6; errs=zeros(1,8);
for j=1:8
 dz=randn(m.ndof,1); dz=dz/norm(dz); dq=m.scale.*dz;
 [~,~,up]=cpr.assemble(m,cpr.retract(m,state,epsFD*dq),zeros(3,1),load);
 [~,~,um]=cpr.assemble(m,cpr.retract(m,state,-epsFD*dq),zeros(3,1),load);
 analytic=r'*dq; numeric=(up-um)/(2*epsFD);
 errs(j)=abs(numeric-analytic)/max(1,abs(analytic));
end
pulleyErr=[]; inverseErr=[];
for j=1:12
 T=cpr.SE3.exp([.5*randn(3,1);.1*randn(3,1)]); rb=.03*randn(3,1);
 [~,K]=cpr.pulley_load(T,rb,cfg.pulley.anchor,.2); D=diag([1 1 1 cfg.L cfg.L cfg.L]); J=zeros(6);
 for col=1:6
  dp=epsFD*D(:,col);
  rp=cpr.pulley_load(T*cpr.SE3.exp(dp),rb,cfg.pulley.anchor,.2);
  rm=cpr.pulley_load(T*cpr.SE3.exp(-dp),rb,cfg.pulley.anchor,.2);
  J(:,col)=(rp-rm)/(2*epsFD);
 end
 pulleyErr(end+1)=norm(J-K*D,'fro')/norm(K*D,'fro'); %#ok<AGROW>
 x=[2.8*rand(1)*randn(3,1)/3;cfg.L*randn(3,1)];
 inverseErr(end+1)=norm(cpr.inverse_jacobian(x,'closed')-cpr.inverse_jacobian(x,'reference'),'fro'); %#ok<AGROW>
end
report=struct('ndof',m.ndof,'maxEnergyGradientError',max(errs),'maxPulleyTangentError',max(pulleyErr),'maxInverseError',max(inverseErr));
assert(report.ndof==258 && max(errs)<1e-5 && max(pulleyErr)<1e-7 && max(inverseErr)<1e-10);
% Failure records and rollback: deliberately allow no Newton updates.
m.cfg.solver.maxIterations=0;
[after,attempts]=cpr.advance(m,state,zeros(3,1),load,[.01;0;0],load);
assert(~attempts(end).info.converged && numel(attempts)>1 && isequaln(after,state));
assert(all(arrayfun(@(a)strcmp(a.info.reason,'maximum_iterations'),attempts)));
report.failedAttemptRollbackPassed=true;
disp(report);
end
