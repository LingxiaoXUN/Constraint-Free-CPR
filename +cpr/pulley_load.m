function [r,K,U,detail]=pulley_load(T,offset,anchor,tension)
%PULLEY_LOAD Rope residual, tangent and potential at the platform origin.
% T maps platform coordinates to world coordinates. offset is the body-fixed
% attachment; anchor is the fixed world guide point. Lengths are in meters
% and tension is in newtons. r and K use right perturbations [omega;v].
R=T(1:3,1:3); p=T(1:3,4); offset=offset(:); anchor=anchor(:);
a=p+R*offset; d=anchor-a; ell=norm(d);
assert(ell>1e-10,'cpr:RopeLength','Rope attachment coincides with guide point.');
u=d/ell; fb=R'*(tension*u); B=[-cpr.SE3.hat3(offset),eye(3)];
H=tension/ell*R'*(eye(3)-u*u')*R;
r=-B'*fb;
% The two terms account for the rotating body frame and changing rope direction.
K=-B'*cpr.SE3.hat3(fb)*[eye(3),zeros(3)]+B'*H*B;
U=tension*ell;
detail=struct('attachment',a,'direction',u,'forceWorld',tension*u,'length',ell);
% K need not be symmetric because the residual is expressed in the moving frame.
end
