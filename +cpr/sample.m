function out=sample(m,state,theta,M)
%SAMPLE Reconstruct rod poses and strains on a uniform material grid.
% Use the fourth-order Magnus expression within each element. out.s is in m;
% position, rotation and strain have sizes 3xMx6, 3x3xMx6 and 6xMx6.
if nargin<4, M=m.cfg.samplesPerRod; end
T=cpr.nodes(m,state,theta); s=linspace(0,m.cfg.L,M);
out.s=s; out.position=zeros(3,M,6); out.rotation=zeros(3,3,M,6); out.strain=zeros(6,M,6);
for k=1:6
 for j=1:M
  e=min(m.N,floor(s(j)/m.h)+1); local=s(j)-(e-1)*m.h; tau=local/m.h;
  [xiL,~,Delta]=cpr.LinStrainNoCont.solveXiL(T(:,:,e,k),T(:,:,e+1,k),state.slope(:,e,k),m.h);
  Om=local*xiL+.5*local*tau*Delta+local^2/12*cpr.SE3.ad(xiL)*(tau*Delta);
  G=T(:,:,e,k)*cpr.SE3.exp(Om);
  out.position(:,j,k)=G(1:3,4); out.rotation(:,:,j,k)=G(1:3,1:3);
  out.strain(:,j,k)=xiL+tau*Delta;
 end
end
end
