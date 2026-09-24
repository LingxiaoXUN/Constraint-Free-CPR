function state=initial_state(m,theta)
%INITIAL_STATE Construct a geometric guess satisfying the rod attachments.
% Set the platform height to 0.8*L. Use cubic centerlines matching the end
% tangents, interpolate the section rotations, and set strain slopes to zero.
% This provides a starting configuration, not an equilibrium solution.
state.platform=m.platformInitial; state.platform(3,4)=.8*m.cfg.L;
state.interior=zeros(4,4,m.N-1,6); state.slope=zeros(6,m.N,6);
base=cpr.base_poses(m,theta);
for k=1:6
 A=base(:,:,k); B=state.platform*m.tipLocal(:,:,k);
 p0=A(1:3,4); p1=B(1:3,4); t0=m.cfg.L*A(1:3,3); t1=m.cfg.L*B(1:3,3);
 rot=cpr.SE3.log([A(1:3,1:3)'*B(1:3,1:3),zeros(3,1);0 0 0 1]);
 for j=1:m.N-1
  u=j/m.N; p=(2*u^3-3*u^2+1)*p0+(u^3-2*u^2+u)*t0+(-2*u^3+3*u^2)*p1+(u^3-u^2)*t1;
  dR=cpr.SE3.exp(u*rot); state.interior(:,:,j,k)=[A(1:3,1:3)*dR(1:3,1:3),p;0 0 0 1];
 end
end
end
