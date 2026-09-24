function [r,K,U]=gravity_load(T,mass,gravity)
%GRAVITY_LOAD Body residual, tangent and potential of a point mass.
% mass is in kg and gravity is a world-frame acceleration in m/s^2.
% The force acts at the origin of the body-to-world pose T.
f=T(1:3,1:3)'*(mass*gravity); r=[zeros(3,1);-f];
K=zeros(6); K(4:6,1:3)=-cpr.SE3.hat3(f);
U=-mass*gravity'*T(1:3,4);
end
