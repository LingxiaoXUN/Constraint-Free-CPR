# Model and implementation

## Conventions

All calculations use SI units. A transform maps a body frame to the world frame. Six-vectors are ordered `[omega; v]`. Pose updates multiply on the right: `T_new = T * exp(hat(dzeta))`. The rod's local z direction is axial. Natural strain is `[0;0;0;0;0;1]`, giving zero natural curvature and a straight, unstretched rod.

## Connection elimination

For rod k, the base pose is obtained by rotating its installation frame about the prescribed motor's fixed world axis. The tip is `T_platform * T_tip_local`. Only platform pose, interior rod poses and strain slopes are stored in a state. `cpr.nodes` constructs the endpoint poses on every residual evaluation.

For a tip perturbation, `dzeta_tip = Ad(inv(T_tip_local)) * dzeta_platform`. An interior perturbation is selected directly and a prescribed base has zero perturbation. `cpr.model` caches each element's participating indices and nonzero mapping blocks P. With prescribed motor angles, these maps are constant in the independent coordinates.

The assembled elastic residual is the sum of `P' * r_element`. The elastic tangent is the sum of `P' * H_element * P`. `cpr.element` retains the source package's two-point Gauss integration, `LinStrainNoCont` strain reconstruction and Jacobians. Its tangent freezes these Jacobians, as in the manuscript; it is not a complete residual Jacobian or a stability Hessian.

## Pulley load

`cpr.pulley_load` evaluates the configuration-dependent rope load and its derivative. The body-fixed rope attachment is r_b, the guide is c, and tension is F0:

```
a = p + R*r_b
ell = norm(c-a)
u = (c-a)/ell
f_b = R'*F0*u
B = [-hat(r_b), I]
H = (F0/ell)*R'*(I-u*u')*R
r_ext = -B'*f_b
K_ext = -B'*hat(f_b)*[I,0] + B'*H*B
```

The first tangent term accounts for the rotating body frame and the second for the changing rope direction. The load potential is F0*ell. Residual and tangent are updated for every Newton and line-search evaluation. The body-residual derivative need not be symmetric and is not forcibly symmetrized.

Rod gravity uses the same lumped mass rule as the original package: half the element mass at each end. Fixed-base contributions affect potential energy but have no free-variable residual. Platform gravity acts at the source model's body origin/center of mass. Gravity also retains the derivative of the body-frame force.

## Initial guess and continuation

The new solver begins with a geometric, non-equilibrium guess: a platform height of 0.8L, cubic rod centerlines matching endpoint tangents, interpolated section frames, and zero initial slopes. It does not load a saved equilibrium. Unlike disconnected straight rods, this guess already satisfies all embedded connections. This is a cold start of the reduced formulation; its iteration count should not be identified with the legacy KKT initialization count.

Initial assembly is solved at the configured initial motor angles, with zero gravity and zero traction. Gravity and, where applicable, tension are then continued to their specified values. Motor actuation subsequently uses the last accepted equilibrium. Failed loading segments are halved and recorded; a failed candidate never replaces the last accepted state. The initial geometric shape and the material's natural strain are distinct concepts.

The Newton direction uses the approximate elastic tangent plus the full external tangent, with numerical regularization and backtracking. The line search evaluates the full potential and residual at the trial configuration. No finite-difference Jacobian is used in production solves.

## Scaling and logging

Let S be diagonal with scales `[1,1,1,L,L,L]` for each pose and `[1/L^2,1/L^2,1/L^2,1/L,1/L,1/L]` for each strain slope. With E0 = EI/L, the solver uses `r_scaled = S*r/E0`, `K_scaled = S*K*S/E0`, and `dq = S*dz`. This is dimensionally consistent energy/coordinate scaling. Convergence requires `norm(r_scaled,inf) < 1e-8` by default.

`info.updates` counts actual accepted Newton updates. `info.evaluations` counts residual/tangent evaluations including line-search trials. History columns are: iteration index before update, scaled residual, potential in joules, accepted line-search step, regularization, accepted flag and elapsed seconds. Per-step timing excludes plotting and file writes. Run-level attempt records include intermediate and rejected loading steps.

## Reconstruction and control outputs

`cpr.sample` uses the source model's partial fourth-order Magnus reconstruction at each requested material coordinate; it does not connect nodes with straight lines. The default is 201 material samples per rod. Saved state and trajectory data use SI units; shape figures display positions in mm.

`CPRRobot` exposes absolute physical motor angles in order [1;2;3]. Motor and load setters update targets. `solve` attempts these targets at the current command time; `run` first calls the controller with `(step,targetTime,robot)`, then solves. A full command commits its state and time only if it reaches the requested target. Otherwise all state changes, including successful intermediate continuation substeps, are rolled back. All attempted solves remain in the log.

`Records` contains accepted commands and the initial equilibrium. `Attempts` records continuation substeps, with a separate flag for acceptance of the full command. `InitializationInfo` stores the geometric cold solve. `info.history` describes the final attempt; total command updates, evaluations and time are summed over attempts, whose individual histories remain available. The public time parameter generates quasi-static commands and introduces no inertial terms.
