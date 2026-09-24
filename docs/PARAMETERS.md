# Parameter and control interface

`robot_parameters.m` returns the complete configuration. Change fields before `CPRRobot(cfg)`; geometry and material are cached by the constructor. All six rods share the following properties.

| Field | Default | Meaning / units |
| --- | --- | --- |
| `N` | 4 | Elements per rod, integer >= 2 |
| `L` | 0.19 | Rod length, m |
| `radius` | 0.0005 | Circular-section radius, m |
| `E` | 1.13e10 | Young's modulus, Pa |
| `nu` | 0.3 | Poisson ratio |
| `rho` | 10 | Rod density, kg/m^3, retained from the prototype script |
| `xiNatural` | `[0;0;0;0;0;1]` | Natural angular strain followed by translational strain; local z is axial |
| `platformMass` | 0.025 | Platform mass, kg |
| `initialMotorAngles` | `[0;0;0]` | Initial physical motor angles, rad |
| `gravity` | `[0;0;-9.81]` | World acceleration, m/s^2 |
| `pulley.anchor` | `[0.1432;0.075;0.150]` | Fixed world guide point, m |
| `pulley.offset` | From geometry | Body-fixed rope attachment, m |
| `pulley.tension` | 0 | Constant rope tension, N |
| `dt` | 0.1 | Default command time interval, s |
| `operator` | `'closed'` | Inverse-Jacobian evaluator: `'closed'`, `'series4'`, `'series6'` |
| `samplesPerRod` | 201 | Material samples used for shape reconstruction |

The stiffness is formed from the circular section, E and G=E/[2(1+nu)], using the source element's axial, shear, bending and torsional terms. Natural angular strain is zero, following the author's correction of the original prototype setting.

## Physical motor numbering

Columns of `axisDirections` and `axisPoints` correspond to motors 1, 2 and 3. Directions must be unit vectors. They are fixed in world coordinates; angles use the right-hand rule.

```matlab
cfg.axisDirections = [[1;0;0], [-.5;-sqrt(3)/2;0], [-.5;sqrt(3)/2;0]];
cfg.axisPoints = [[0;-.03;-.01], [-.03*sqrt(3)/2;.015;0], [.03*sqrt(3)/2;.015;0]];
```

These axis points retain the source prototype's first-motor z offset. They can be changed as physical geometry parameters. `commandForMotor=1:3` is fixed by the new public API, so `setMotorAngle(2,...)` always addresses physical motor 2. To reuse the old prototype's reversed command assignment, explicitly reorder the old command: `oldTheta([3,2,1])`.

## Attachment geometry

`cpr.default_geometry` reads the numeric installation transforms in `data/geometry.json`. All transforms map the named local frame into its parent frame and use meters.

| Field within `cfg.geometry` | Size | Meaning |
| --- | --- | --- |
| `platformInitial` | 4x4 | Source platform installation pose |
| `motorInitial` | 4x4x3 | Motor installation poses in world coordinates |
| `baseLocal` | 4x4x6 | Rod base attachments relative to their motor |
| `tipLocal` | 4x4x6 | Rod tip attachments relative to the platform |
| `motorForRod` | Six entries | Parent motor index for each rod, two rods per motor |
| `pulleyOffset` | 3x1 | Default body-fixed rope attachment |

The active rope offset is `cfg.pulley.offset`, which overrides the geometry default. Updating `geometry.pulleyOffset` alone after `robot_parameters` does not change the active offset.

The geometric cold guess uses the installation platform's rotation and lateral position, but sets its height to `0.8*L`; `platformInitial` is not a prescribed platform boundary condition. The free platform pose is solved. Attachment transforms were extracted from the original constraint definitions, not fitted to a trajectory.

## Solver and visualization

`solver` fields: `tolerance=1e-8`, `maxIterations=180`, `maxBacktracks=18`, `minStep=2^-18`, `maxScaledStep=0.5`, `verbose=false`. Runtime changes use `robot.setSolverOptions(struct('tolerance',1e-9))`. The tolerance applies to the dimensionless scaled residual defined in `MODEL.md`. A failed continuation segment is bisected down to `continuation.minFraction=1/128` of the requested segment.

`visualization.enabled=false` disables automatic plotting in `run`; `visualization.every=1` plots every accepted step when enabled; `visualization.showMotorLabels=true` labels the physical motors. Manual `robot.plot` is always available after initialization.

| Visualization field | Default | Meaning |
| --- | --- | --- |
| `view` | `[38 24]` | Azimuth and elevation, degrees |
| `lockView` | `true` | Fixed camera; disable mouse rotation, zoom and pan |
| `axisLimits` | `[]` | Compute limits once, or supply `[xmin xmax; ymin ymax; zmin zmax]` in m |
| `motorWidth` | 0.026 | Display rectangle width, m |
| `motorHeight` | 0.010 | Display motor thickness, m |
| `motorEndPadding` | 0.006 | Display length added beyond each rod attachment, m |
| `platformThickness` | 0.004 | Display platform thickness, m |
| `rodLineWidth` | 2.6 | Rod stroke width, points |

Motor rectangles are constructed in the motor's local frame around its two rod attachments and transformed by the actual motor pose. The platform plate follows the platform pose. These are schematic display shapes and do not modify stiffness, mass or attachment locations. Set `lockView=false` to rotate interactively. For trajectories outside the default plot range, supply larger fixed `axisLimits`.

## Runtime controls

| Method | Effect |
| --- | --- |
| `setMotorAngles(theta)` | Three absolute target angles in rad |
| `setMotorAngle(id,angle)` | One absolute target; preserves the other targets |
| `setPulleyTension(F0)` | Target tension in N, nonnegative |
| `setGravity(g)` | Target world acceleration in m/s^2 |
| `setCallback(@controller)` | Controller signature `(step,targetTime,robot)` |
| `solve()` | Solve current targets at the current time |
| `run(nSteps,dt)` | Call the controller and solve sequential targets |
| `getPlatformPose()` | Last accepted body-to-world transform |

Setters do not move the accepted state. The callback can inspect the previous accepted equilibrium before setting targets. `dt` is a quasi-static command parameter, not a dynamics time step. `setCallback([])` disables the callback.

`legacy_fixtures.mat` contains 15 numeric states from the original explicit-constraint solver for regression testing. Production runs do not read these states as initial guesses.
