# precice-mbdyn-coupling-tests
Tests to couple MBDyn with PreCICE

## List of tests:

1) *rigid_motion* : an airfoil is rotated with specific pattern and forces are measured.

commands:
- in *fluid* SU2_CFD rigid_config.cfg   (assuming SU2_CFD in path)
- in *rigid* ../coupling-code/bin/RigidMotion ../config.xml NACA_0012.foil  (assuming that coupling code has been compiled)

