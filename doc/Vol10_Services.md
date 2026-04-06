# Volume 10: 坐标变换服务 (Coordinate Service)

在复杂的 GNC 仿真中，坐标系的转换极易出错。从 ECEF 到 Body 系，中间可能经过 NED，又或者是 ECI $\rightarrow$ ECEF $\rightarrow$ Launch $\rightarrow$ Target。

## 1. 痛点与传统做法
如果每个组件自己推导转换矩阵：
```cpp
Matrix3 R_ecef_ned = ecef_to_ned_rotation(lat, lon);
Matrix3 R_ned_body = attitude.toRotationMatrix();
Vector3 v_body = R_ned_body * R_ecef_ned * v_ecef;
```
一旦加入更多的坐标系，代码会变得冗长且难以维护。

## 2. 树形坐标管理 (`CoordinateService`)
`gnc::services::CoordinateService` 是一个全局服务，您可以通过 `injectServices` 获取。它将所有坐标系组织成一棵严格的树。

### 2.1 注册拓扑关系
在仿真初始化时，您可以注册坐标系之间的“变换提供者”（Lambda 函数）：
```cpp
coord_service_->setRoot(FrameId::ECI);

coord_service_->registerTransform(FrameId::ECEF, FrameId::ECI,
    [&]() { return eci_to_ecef_rotation(current_gast); });

coord_service_->registerTransform(FrameId::NED, FrameId::ECEF,
    [&]() { return ecef_to_ned_rotation(lat, lon); });

coord_service_->registerTransform(FrameId::BODY, FrameId::NED,
    [&]() { return attitude_.toRotationMatrix(); });
```

### 2.2 自动路径查找 (LCA)
当业务组件需要计算从 ECI 到 BODY 的转换时：
```cpp
Matrix3 R = coord_service_->getRotation(FrameId::BODY, FrameId::ECI, sim_time);
Vector3 v_eci = coord_service_->transform(v_body, FrameId::BODY, FrameId::ECI, sim_time);
```
**内部算法**：
- `CoordinateService` 会在树中自动寻找 `BODY` 和 `ECI` 的**最近公共祖先 (LCA)**。
- 然后它会自动沿着路径，将正向变换（左乘）和逆向变换（转置矩阵左乘）组合在一起，得到最终的 $M_{BODY \rightarrow ECI}$。

### 2.3 节点级缓存机制
由于 `RotationProvider` 内部可能涉及大量的三角函数计算（`sin`, `cos`），在同一个时间步内（`sim_time` 相同），服务会缓存每个节点的旋转矩阵。
这使得 10 个不同的组件在同一帧请求 `ECEF -> BODY` 的变换时，底层的 `sin/cos` 只会计算一次！

## 3. 在仿真构建中的启用
要在 JSON 中启用并注入 `CoordinateService`：
```json
"services": {
    "coordinate": { "enabled": true }
}
```
框架的 `SimulationBuilder` 会自动将其挂载到 `ServiceContext` 中。