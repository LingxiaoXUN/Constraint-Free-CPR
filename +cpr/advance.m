function [state,attempts]=advance(m,state,theta0,load0,theta1,load1)
%ADVANCE Continue angles and loads linearly from the old to the new target.
% Bisect failed segments down to minFraction. state is the last converged
% substep; attempts records both successes and failures. The caller decides
% whether to retain a partial result when the full target is not reached.
fraction=0; step=1; attempts=struct([]);
while fraction<1-1e-14
 target=min(1,fraction+step);
 theta=(1-target)*theta0+target*theta1;
 load=struct('gravity',(1-target)*load0.gravity+target*load1.gravity,'tension',(1-target)*load0.tension+target*load1.tension);
 [candidate,info]=cpr.solve(m,state,theta,load);
 rec=struct('fromFraction',fraction,'targetFraction',target,'theta',theta,'load',load,'info',info);
 attempts=[attempts,rec]; %#ok<AGROW>
 if info.converged
  state=candidate; fraction=target; step=min(1,step*1.5);
 else
  step=step/2;
  if step<m.cfg.continuation.minFraction, return; end
 end
end
end
