function [base,motors]=base_poses(m,theta)
%BASE_POSES Motor and rod-base poses for absolute motor angles in radians.
% Rotate each installation pose about its fixed world axis, then apply the
% local rod attachments. commandForMotor maps physical motors to input entries.
base=zeros(4,4,6); motors=zeros(4,4,3);
for j=1:3
 a=m.cfg.axisDirections(:,j); c=m.cfg.axisPoints(:,j);
 xi=[a;-cross(a,c)]; G=cpr.SE3.exp(theta(m.cfg.commandForMotor(j))*xi);
 motors(:,:,j)=G*m.motorInitial(:,:,j);
end
for k=1:6, base(:,:,k)=motors(:,:,m.motorForRod(k))*m.baseLocal(:,:,k); end
end
