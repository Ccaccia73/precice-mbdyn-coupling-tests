# precice-mbdyn-coupling-tests
Tests to couple MBDyn with PreCICE

## List of tests:

1) *rigid_motion* : an airfoil is moved with specific pattern and forces are measured. First test, might be incomplete, better use the next one.

commands:
- in *fluid* SU2_CFD rigid_config.cfg   (assuming SU2_CFD in path)
- in *rigid* ../coupling-code/bin/RigidMotion ../config.xml NACA_0012.foil  (assuming that coupling code has been compiled)

2) *0012_points* : a NACA0012 airfoil is rotated with the pattern shown in the graph and forces are measured. the CL is computed and compared to theoric. Different number of points is considered for the rigid mesh

- *fluid-start* : is used to create a first solution to be used in the fluid solver. **SU2_CFD force_start_config.cfg** and copy **restart_flow.dat** into *fluid* as **restart_flow_00000.dat**
- in *fluid* :  SU2_CFD rigid:config.cfg 
- in *rigid* :  the executable is the same of the first test: RigidMotion ../config.xml /foils/NACA_0012_XXX.foil

3) *ts-driver* : test to set external time step to MBDyn

- in one shell start MBDyn form the mbd directory: mbdyn -f input.mbd - o output
- in the other shell run ./bin/ts_driver

4) *mbdyn-external*: test to use an external node:

- from the mbd subdirectory launch mbdyn -f spring1.mbd
- form the bin directory launch ./mbdyn_external
- the output file "Forces.txt" is analysed in the notebook subdirectory 