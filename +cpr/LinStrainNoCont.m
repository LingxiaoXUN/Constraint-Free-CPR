classdef LinStrainNoCont
%LINSTRAINNOCONT Element strain reconstruction and pose sensitivities.
% Piecewise-linear strain per element (no inter-element continuity enforced).
% Variables per element: end poses (global nodal T_i, T_{i+1}) and slope a (6x1).
% Order convention: xi=[w; v], right-multiplicative updates on SE(3).

methods (Static)

    function [xiL, xiR, Delta, ell, g, A] = solveXiL(Ti, Tj, a, h)
        % Inputs:
        %   Ti,Tj : 4x4 end poses of element
        %   a     : 6x1 strain slope on element (constant)
        %   h     : reference element length, m
        % Outputs:
        %   xiL   : 6x1 left-end strain
        %   xiR   : 6x1 right-end strain
        %   Delta : 6x1 = h*a
        %   ell   : 6x1 = vee(log(g)), g=Ti^{-1}Tj
        %   g     : relative pose
        %   A     : 6x6 coefficient matrix relating xiL to the relative pose


        g   = cpr.SE3.inv(Ti) * Tj;
        ell = cpr.SE3.log(g);
        Delta = h * a;

        % A(Delta) * xiL = ell - (h/2) * Delta
        A = h*eye(6) - (h^2/12) * cpr.SE3.ad(Delta);
        rhs = ell - 0.5*h*Delta;
        xiL = A \ rhs;
        xiR = xiL + Delta;
    end

    function [taus, wq] = quadNodesWeights(h)
        % 2-point Gauss on [0,h] using tau in [0,1]
        taus = [0.5 - 1/(2*sqrt(3)), 0.5 + 1/(2*sqrt(3))];
        wq   = (h/2) * [1, 1];
    end

    function Xi_q = quadStrain(xiL, Delta, h)
        % Return 6x2 strains at Gauss points inside the element
        [taus,~] = cpr.LinStrainNoCont.quadNodesWeights(h);
        Xi_q = zeros(6,2);
        for k=1:2
            Xi_q(:,k) = xiL + taus(k)*Delta;
        end
    end

    function Kerns = kernels(Ti, Tj, a, h, mode_dexp)
        % Build element-level sensitivity kernels for assembly.
        % Outputs in struct Kerns with fields:
        %   .xiL, .xiR, .Delta, .ell, .g
        %   .S_etaL_q{2}, .S_etaR_q{2}, .S_a_q{2}   (each 6x6)
        %
        % mode_dexp: 'closed', 'series4', or 'series6'

        if nargin < 5, mode_dexp = 'closed'; end

        import cpr.SE3.*

        % Recover end strains from the relative end pose and strain slope.
        [xiL, xiR, Delta, ell, g, A] = cpr.LinStrainNoCont.solveXiL(Ti, Tj, a, h);

        % Inverse right Jacobian and adjoint of the relative end pose.
        Minv  = cpr.inverse_jacobian(ell, mode_dexp);          % 6x6
        Aginv = cpr.SE3.Ad(cpr.SE3.inv(g));                    % 6x6
        ad_xiL= cpr.SE3.ad(xiL);

        % Jacobians for xiL:
        % dxiL/detaL = -A^{-1} * Minv * Aginv
        % dxiL/detaR =  A^{-1} * Minv
        % dxiL/da    =  -A^{-1} * ( (h^2/2) I + (h^3/12) ad_{xiL} )
        J_etaL = - A \ (Minv * Aginv);
        J_etaR =   A \ Minv;
        J_a    = - A \ ( (h^2/2)*eye(6) + (h^3/12)*ad_xiL );

        % Two-point Gauss quadrature along the reference element.
        [taus, wq] = cpr.LinStrainNoCont.quadNodesWeights(h);
        Xi_q = cpr.LinStrainNoCont.quadStrain(xiL, Delta, h);

        % Strain Jacobians at Gauss points:
        % xi_q = xiL + tau * Delta
        % => dxi_q/detaL = J_etaL
        %    dxi_q/detaR = J_etaR
        %    dxi_q/da    = J_a + tau * h * I
        S_etaL_q = cell(1,2); S_etaR_q = cell(1,2); S_a_q = cell(1,2);
        I6 = eye(6);
        for k=1:2
            tau = taus(k);
            S_etaL_q{k} = J_etaL;
            S_etaR_q{k} = J_etaR;
            S_a_q{k}    = J_a + tau * h * I6;
        end

        % Bundle strains and sensitivities for element assembly.
        Kerns.xiL   = xiL;
        Kerns.xiR   = xiR;
        Kerns.Delta = Delta;
        Kerns.ell   = ell;
        Kerns.g     = g;

        Kerns.Xi_q     = Xi_q;     % 6x2 strains
        Kerns.taus     = taus;     % 1x2
        Kerns.wq       = wq;       % 1x2

        Kerns.S_etaL_q = S_etaL_q; % cell{2} of 6x6
        Kerns.S_etaR_q = S_etaR_q; % cell{2} of 6x6
        Kerns.S_a_q    = S_a_q;    % cell{2} of 6x6
    end

end
end
