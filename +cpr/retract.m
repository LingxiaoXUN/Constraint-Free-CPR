function out=retract(m,state,dq)
%RETRACT Apply an increment in the independent coordinates.
% Pose increments multiply on the right; strain slopes update additively.
% dq is dimensional: any solver scaling must already have been applied.
out=state; out.platform=state.platform*cpr.SE3.exp(dq(1:6));
for k=1:6
 for j=1:m.N-1, out.interior(:,:,j,k)=state.interior(:,:,j,k)*cpr.SE3.exp(dq(m.interiorIds(:,j,k))); end
 for e=1:m.N, out.slope(:,e,k)=state.slope(:,e,k)+dq(m.slopeIds(:,e,k)); end
end
end
