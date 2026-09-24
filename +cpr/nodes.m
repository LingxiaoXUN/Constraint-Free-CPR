function T=nodes(m,state,theta)
%NODES Reconstruct the 4-by-4-by-(N+1)-by-6 array of rod node poses.
% Endpoints follow their motor or platform attachment. Interior poses are
% read from state; every pose maps the section frame to the world frame.
base=cpr.base_poses(m,theta); T=zeros(4,4,m.N+1,6);
for k=1:6
 T(:,:,1,k)=base(:,:,k); T(:,:,end,k)=state.platform*m.tipLocal(:,:,k);
 T(:,:,2:m.N,k)=state.interior(:,:,:,k);
end
end
