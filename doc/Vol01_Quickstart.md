# Volume 1: 快速上手与环境搭建 (Quickstart)

## 1. 简介
欢迎使用 GNC 仿真框架 v2.0！本手册将手把手带您从零开始配置环境、编译代码，并运行第一个仿真案例。

## 2. 环境依赖与安装 (Linux / Ubuntu)

本框架使用 C++17 编写，构建系统依赖 `CMake`，核心矩阵运算唯一依赖 `Eigen3` 库。

请打开终端，执行以下命令安装必备环境：
```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake python3 python3-pip
```

**安装 Eigen3**:
由于 Eigen3 是一个纯头文件库，您有两种方式安装：

*方式一：通过包管理器（推荐）*
```bash
sudo apt-get install -y libeigen-dev
```

*方式二：手动源码安装（如果包管理器失败或需要特定版本）*
```bash
wget https://gitlab.com/libeigen/eigen/-/archive/3.3.9/eigen-3.3.9.tar.gz
tar -xzf eigen-3.3.9.tar.gz
cd eigen-3.3.9
mkdir build && cd build
cmake ..
sudo make install
cd ../..
```

*(如果您需要运行可视化脚本，请安装 matplotlib)*
```bash
pip3 install matplotlib
```

## 3. 编译框架

框架使用了高度自动化的 CMake 构建系统。它会自动扫描 `user/components/` 下的源文件并进行注册，您**无需手动修改 `CMakeLists.txt`**。

在项目的根目录下（即 `CMakeLists.txt` 所在的目录），执行以下命令：
```bash
# 1. 创建并进入构建目录
mkdir -p build && cd build

# 2. 生成 Makefile (Release 模式性能最好)
cmake .. -DCMAKE_BUILD_TYPE=Release

# 3. 编译代码
cmake --build . --config Release
```

编译成功后，您将在 `build/bin/` 目录下看到生成的可执行文件，包括主程序 `gnc_sim` 和一些示例程序（如 `example_cavh_3dof`）。

## 4. 运行第一个仿真案例

我们来运行高超声速飞行器 (CAVH) 的 3DOF 仿真。该案例的配置文件位于 `examples/03_cavh_3dof/cavh_mission.json`。

退回到项目根目录，执行：
```bash
cd ..
./build/bin/example_cavh_3dof
```

### 您将看到类似如下的终端输出：
```text
[INFO] Simulation config: dt=0.1, duration=600
[INFO] Dependency validation passed
[INFO] AutoDataLogger enabled with 12 field(s). Output directory: 'user/outputs/2026-04-06_074822'
[INFO] Starting simulation: dt=0.1, duration=600
[INFO] Simulation terminated early by condition 'Altitude below 10km' at t=527.2 s, step=5271
[INFO] Simulation completed. Final time: 527.2, reason: Altitude below 10km
```
*这表示仿真成功运行！由于我们配置了安全停止条件，飞行器在下降到 10km 高度（第 527.2 秒）时，仿真自动安全终止。*

## 5. 结果可视化

仿真产生的数据被自动保存在 `user/outputs/` 的最新时间戳目录下（例如 `user/outputs/2026-04-06_074822/cavh_3dof.csv`）。

我们可以使用官方脚本来查看结果。在项目根目录下执行：
```bash
python3 tools/plot_results.py user/outputs/2026-04-06_074822/cavh_3dof.csv
```
*(注意替换为您实际生成的时间戳文件夹名)*

如果没有安装 `matplotlib`，脚本将在终端打印纯文本的极值统计：
```text
============================================================
  Gravity Turn Simulation Summary
============================================================
  Total time points: 5273
  Time range: 0.00 - 527.20 s
  dynamics.altitude          : final =    10000.000  min =    10000.000  max =    60000.000
  dynamics.velocity          : final =     1500.000  min =     1500.000  max =     3200.000
============================================================
```
如果安装了 `matplotlib`，它将生成一张包含轨迹、速度、姿态指令的高清 `PNG` 图片。

---
**恭喜！您已经成功掌握了 GNC 框架的编译与运行全流程。**
接下来，请阅读 `Vol02` 了解框架的五层架构与主动拉取模型。