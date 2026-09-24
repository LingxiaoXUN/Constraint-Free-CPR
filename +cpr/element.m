function [r,H,energy,kerns]=element(Ta,Tb,beta,h,K,xiNatural,mode)
%ELEMENT Elastic residual, approximate tangent and energy of one rod element.
% Ta and Tb are end poses; beta is the constant six-component strain slope.
% h is the reference length in meters, K is the 6-by-6 sectional stiffness,
% and xiNatural is ordered [angular strain; translational strain].
% Local increments are [left pose; right pose; strain slope].
kerns=cpr.LinStrainNoCont.kernels(Ta,Tb,beta,h,mode);
r=zeros(18,1); H=zeros(18); energy=0;
for q=1:2
 S=[kerns.S_etaL_q{q},kerns.S_etaR_q{q},kerns.S_a_q{q}];
 dx=kerns.Xi_q(:,q)-xiNatural; w=kerns.wq(q);
 r=r+w*S'*(K*dx); H=H+w*S'*K*S; energy=energy+.5*w*dx'*K*dx;
end
% H retains the S'*K*S terms and omits derivatives of the strain Jacobians.
% It is the Gauss-Newton approximation, not a tangent for stability analysis.
end
