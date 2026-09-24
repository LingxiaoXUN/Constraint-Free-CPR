function M=inverse_jacobian(x,mode)
%INVERSE_JACOBIAN Inverse right Jacobian on SE(3), ordered [omega;v].
% M = d Log(Exp(x)*Exp(eta))/d eta at eta=0.
% Modes are 'closed', 'series4', 'series6', and 'reference'. The reference
% uses a block matrix exponential for independent numerical checks.
A=cpr.SE3.ad(x); A2=A*A; A4=A2*A2;
switch char(mode)
case 'series4'
    M=eye(6)+A/2+A2/12-A4/720;
case 'series6'
    M=eye(6)+A/2+A2/12-A4/720+A4*A2/30240;
case 'closed'
    t=norm(x(1:3)); t2=t*t;
    if t<0.1
        % Stable small-angle coefficient limit of the closed polynomial.
        % A^5+2*t^2*A^3+t^4*A=0 reduces the even Bernoulli powers.
        b=[1/12,-1/720,1/30240,-1/1209600,1/47900160,-691/1307674368000];
        c=[b(1);0]; p=[0;1];
        for k=2:numel(b)
            c=c+b(k)*p;
            p=[-t2*t2*p(2);p(1)-2*t2*p(2)];
        end
        c2=c(1); c4=c(2);
    else
        h=(t/2)*cot(t/2); q=(t/2/sin(t/2))^2;
        c2=(4-3*h-q)/(2*t2); c4=(2-h-q)/(2*t2*t2);
    end
    M=eye(6)+A/2+c2*A2+c4*A4;
case 'reference'
    % Upper-right block is integral_0^1 exp(-s A) ds; no inversion of A.
    B=expm([-A,eye(6);zeros(6,12)]); M=B(1:6,7:12)\eye(6);
otherwise
    error('Unknown audit method');
end
end
