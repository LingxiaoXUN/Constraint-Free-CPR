function report=test_legacy_fixtures
%TEST_LEGACY_FIXTURES Compare with equilibria from the explicit-constraint solver.
root=fileparts(fileparts(mfilename('fullpath')));addpath(root);
d=load(fullfile(root,'data','legacy_fixtures.mat')); f=d.fixtures;
maxPosition=0;maxRotation=0;maxAttachment=0;maxResidual=0;
for j=1:numel(f)
 m=cpr.model(f(j).cfg);theta=f(j).theta;target=f(j).state;
 % Perturb each stored equilibrium and check that the solver returns to it.
 % The fixture data were generated with explicit attachment constraints.
 dz=sin((1:m.ndof)')*1e-5;
 initial=cpr.retract(m,target,m.scale.*dz);
 [st,info]=cpr.solve(m,initial,theta,f(j).load);assert(info.converged);
 maxResidual=max(maxResidual,info.residualScaled);
 maxPosition=max(maxPosition,1000*norm(st.platform(1:3,4)-target.platform(1:3,4)));
 xi=cpr.SE3.log(cpr.SE3.inv(target.platform)*st.platform);maxRotation=max(maxRotation,norm(xi(1:3)));
 T=cpr.nodes(m,st,theta);
 for k=1:6,maxAttachment=max(maxAttachment,norm(T(:,:,end,k)-st.platform*m.tipLocal(:,:,k),'fro'));end
 sm=cpr.sample(m,target,theta,numel(f(j).sample.s));
 assert(max(abs(sm.position(:)-f(j).sample.position(:)))<1e-12,'Reconstruction regression');
end
report=struct('fixtureCount',numel(f),'maxPlatformPosition_mm',maxPosition,'maxPlatformRotation_rad',maxRotation,'maxAttachmentError',maxAttachment,'maxScaledResidual',maxResidual);
assert(maxPosition<1e-4 && maxRotation<1e-5 && maxAttachment<1e-12);disp(report);
end
