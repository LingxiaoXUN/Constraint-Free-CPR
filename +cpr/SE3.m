classdef SE3
%SE3 Rigid-transform operations with rotation-first twist coordinates.
%  All 6-vectors follow xi = [w; v], with w the rotation part (so(3)),
%  v the translation part (R^3). Right update: T_new = T * exp(xi).

methods (Static)

    function tilde_xi = tilde(xi)
        % Map [omega;v] to a 4-by-4 twist matrix.
        a = xi(1:3);
        b = xi(4:6);
        W = cpr.SE3.hat3(a);
        tilde_xi = [W b ; zeros(1,4)];
    end

    function xi = invtilde(tilde_xi)
        % Extract [omega;v] from a 4-by-4 twist matrix.
        A = tilde_xi(1:3,1:3);
        b = tilde_xi(1:3,4);
        a = cpr.SE3.vee3(A);
        xi = [a ; b];
    end

    function W = hat3(a)
        % Return the skew matrix satisfying W*b = cross(a,b).
        W = [   0   -a(3)  a(2);
              a(3)    0   -a(1);
             -a(2)  a(1)    0  ];
    end

    function a = vee3(W)
        % Extract the axial vector of a 3-by-3 skew matrix.
        a = [W(3,2); W(1,3); W(2,1)];
    end

    function Ad = Ad(T)
        % Ad_T in [w; v] ordering
        R = T(1:3,1:3); p = T(1:3,4);
        Ad = [ R, zeros(3); cpr.SE3.hat3(p)*R, R ];
    end

    function Tinv = inv(T)
        % Inverse rigid transform: [R', -R'*p; 0 0 0 1].
        R = T(1:3,1:3); p = T(1:3,4);
        Tinv = [R.', -R.'*p; 0 0 0 1];
    end

    function T = exp(xi)
        % Matrix exponential of the 4-by-4 twist matrix.
        tilde_xi = cpr.SE3.tilde(xi);
        T = expm(tilde_xi);
    end

    function xi = log(T)
        % Matrix logarithm, returned as a six-component twist.

        tilde_xi = logm(T);
        xi = cpr.SE3.invtilde(tilde_xi);      
    end

    function w = logSO3(R)
        % Rotation vector from a 3-by-3 rotation matrix.
        [U,~,V] = svd(R);
        R = U*V';
        c = max(-1,min(1,(trace(R)-1)/2));
        th = acos(c);
        if th < 1e-50
            w = [0;0;0];
        else
            W = (R - R.')/(2*sin(th));
            w = cpr.SE3.vee3(W) * th;
        end
    end

    function ad = ad(xi)
        % ad_xi in [w; v]
        w = xi(1:3); v = xi(4:6);
        W = cpr.SE3.hat3(w); V = cpr.SE3.hat3(v);
        ad = [ W, zeros(3); V, W ];
    end

end
end
