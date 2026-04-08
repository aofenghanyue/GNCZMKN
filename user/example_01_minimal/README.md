# example_01_minimal

这是统一 `user/` 工程区下的最小工程示例。

1. 将 `user/active_project` 设为 `example_01_minimal`
2. 重新构建 `gnc_sim`
3. 不带 `--config` 直接运行 `gnc_sim`

```powershell
Set-Content user\active_project example_01_minimal -NoNewline
cmake --build build-mingw --config Release --target gnc_sim
.\build-mingw\bin\gnc_sim.exe
```
