# example_05_coordinate_service_local_geographic

这个示例只演示内建苏式体系里的动态 `NUE`：

- `ECI`
- `ECEF`
- `LAUNCH`
- `LAUNCH_INERTIAL`
- `NUE`

这里的 `NUE` 由一个用户侧适配 provider 提供经纬高 `[lat_rad, lon_rad, alt_m]`。
这正是框架当前要求的内建苏式语义：如果现有 provider 的输出语义不匹配，就写一个适配 provider，而不是给内建体系增加额外开关。

运行方式：

1. 将 `user/active_project` 设为 `example_05_coordinate_service_local_geographic`
2. 重新构建 `gnc_sim`
3. 运行本示例 mission

```powershell
Set-Content user\active_project example_05_coordinate_service_local_geographic -NoNewline
cmake --build build-cmake-mingw-soviet-rebuild --target gnc_sim
.\build-cmake-mingw-soviet-rebuild\bin\gnc_sim.exe --config user/example_05_coordinate_service_local_geographic/config/mission.json
```
