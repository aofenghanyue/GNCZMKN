# example_04_coordinate_service_soviet

这个示例只演示框架内建苏式坐标体系的固定上半层：

- `ECI`
- `ECEF`
- `LAUNCH`
- `LAUNCH_INERTIAL`

它不演示动态 `NUE/BODY/TRACK/WIND`，目的是把“苏式体系固定定义”先单独跑通。

运行方式：

1. 直接用 `--config` 指向本示例的 mission 文件。
2. 不需要切换 `user/active_project`，因为这个示例只用 starter components。

```powershell
.\build-cmake-mingw-soviet-rebuild\bin\gnc_sim.exe --config user/example_04_coordinate_service_soviet/config/mission.json
```
