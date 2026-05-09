# CAVH Geographic 3DOF Custom Project

This user project keeps the legacy CAVH geographic 3DOF demonstration outside
the framework built-ins. It uses the current framework flow contracts and adds
only the CAVH-specific pieces as project components:

- programmed angle-of-attack guidance;
- CSV-backed 2D aerodynamic assets;
- aero-propulsive local-spherical 3DOF interaction.

Build it as an active project:

```powershell
cmake -S . -B build-mingw-cavh -G "MinGW Makefiles" -DBUILD_TESTS=ON -DGNC_ACTIVE_PROJECT=example_08_cavh_geographic_3dof_custom
cmake --build build-mingw-cavh -j 4
build-mingw-cavh\bin\gnc_sim.exe --config user\example_08_cavh_geographic_3dof_custom\config\mission.json
```
