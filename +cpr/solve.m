function [state,info]=solve(m,state,theta,load)
%SOLVE Find equilibrium for fixed motor angles and loads.
% Use a scaled approximate Newton direction with regularization and energy
% backtracking. info reports convergence, accepted updates and trial evaluations.
opt=m.cfg.solver; t0=tic; S=m.scale; E0=m.energyScale;
hist=zeros(0,7); updates=0; evaluations=0; status='maximum_iterations'; mu=1e-9;
[r,K,U]=cpr.assemble(m,state,theta,load); evaluations=evaluations+1;
% Scale pose and strain-slope components by S, then normalize by energy E0.
for it=0:opt.maxIterations
 rs=S.*r/E0; normR=norm(rs,inf);
 if normR<opt.tolerance, status='converged'; break; end
 if it==opt.maxIterations, break; end
 Ks=(S.*K).*S'/E0; accepted=false; alpha=0;
 for dampingTry=1:9
  dz=-(Ks+mu*eye(m.ndof))\rs;
  stepSize=max(abs(dz)); if stepSize>opt.maxScaledStep, dz=dz*(opt.maxScaledStep/stepSize); end
  descent=rs'*dz;
  if ~all(isfinite(dz)) || descent>=0, mu=max(1e-6,10*mu); continue; end
  alpha=1;
  for backtrack=0:opt.maxBacktracks
   % Reassemble at each trial pose so the rope direction follows the platform.
   trial=cpr.retract(m,state,alpha*S.*dz);
   [rt,Kt,Ut]=cpr.assemble(m,trial,theta,load); evaluations=evaluations+1;
   trialNorm=norm(S.*rt/E0,inf);
   decrease=(Ut-U)/E0;
   if isfinite(Ut) && (decrease<=1e-4*alpha*descent || (abs(decrease)<1e-12 && trialNorm<normR))
    accepted=true; break;
   end
   alpha=alpha/2; if alpha<opt.minStep, break; end
  end
  if accepted, break; end
  mu=max(1e-6,10*mu);
 end
 hist(end+1,:)=[it,normR,U,alpha,mu,accepted,toc(t0)]; %#ok<AGROW>
 if opt.verbose, fprintf('it %d scaled_res %.3g alpha %.3g damping %.3g\n',it,normR,alpha,mu); end
 if ~accepted, status='line_search_failed'; break; end
 state=trial; r=rt; K=Kt; U=Ut; updates=updates+1;
 if alpha>=.5, mu=max(1e-12,mu*.2); end
end
info=struct('converged',strcmp(status,'converged'),'reason',status,'updates',updates,...
 'evaluations',evaluations,'residualScaled',norm(S.*r/E0,inf),'energyJ',U,'seconds',toc(t0),...
 'history',hist,'tolerance',opt.tolerance);
end
