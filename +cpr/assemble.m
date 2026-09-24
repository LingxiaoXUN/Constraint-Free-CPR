function [r,K,U,detail]=assemble(m,state,theta,load)
%ASSEMBLE Elastic and external residuals in independent coordinates.
% r is ndof-by-1, K is ndof-by-ndof, and U is the total potential in joules.
% The elastic block uses a Gauss-Newton tangent; external blocks retain
% their full derivatives under right pose perturbations.
if nargin<4, load=struct('gravity',m.cfg.gravity,'tension',m.cfg.pulley.tension); end
T=cpr.nodes(m,state,theta); r=zeros(m.ndof,1); K=zeros(m.ndof); U=0;
fullRodResidual=zeros(6,m.N+1,6);
for k=1:6
 for e=1:m.N
  [re,Ke,Ue]=cpr.element(T(:,:,e,k),T(:,:,e+1,k),state.slope(:,e,k),m.h,m.K,m.cfg.xiNatural,m.cfg.operator);
  fullRodResidual(:,e,k)=fullRodResidual(:,e,k)+re(1:6);
  fullRodResidual(:,e+1,k)=fullRodResidual(:,e+1,k)+re(7:12);
  % Lump half the element mass at each endpoint.
  for j=1:2
   [rg,Kg,Ug]=cpr.gravity_load(T(:,:,e+j-1,k),m.massPerLength*m.h/2,load.gravity);
   ii=(j-1)*6+(1:6); re(ii)=re(ii)+rg; Ke(ii,ii)=Ke(ii,ii)+Kg; Ue=Ue+Ug;
  end
  P=m.maps{e,k}; ids=m.indices{e,k};
  % P maps independent increments to the element coordinates.
  r(ids)=r(ids)+P'*re; K(ids,ids)=K(ids,ids)+P'*Ke*P; U=U+Ue;
 end
end
[rg,Kg,Ug]=cpr.gravity_load(state.platform,m.cfg.platformMass,load.gravity);
% Recompute the rope direction and its derivative at this configuration.
[rp,Kp,Up,rope]=cpr.pulley_load(state.platform,m.pulleyOffset,m.cfg.pulley.anchor,load.tension);
r(1:6)=r(1:6)+rg+rp; K(1:6,1:6)=K(1:6,1:6)+Kg+Kp; U=U+Ug+Up;
detail=struct('nodes',T,'rope',rope,'elasticNodeResidual',fullRodResidual);
end
