# 最小示例

1. 将 `components/constant_guidance.hpp` 复制到 `user/components/guidance/`
2. 将 `config/mission.json` 复制到 `user/config/missions/`
3. 重新构建并运行

```powershell
cd build
cmake ..
cmake --build . --config Release
cd ..
.\build\bin\gnc_sim.exe user\config\missions\mission.json
```
