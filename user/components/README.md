# 用户组件目录

将你的自定义组件放在此目录或其子目录下。

## 建议结构

```text
components/
├── guidance/
├── controller/
├── navigation/
└── your_component.hpp
```

## 规则

1. 每个组件使用一个 `.hpp` 文件
2. 继承 `ComponentBase` 和对应接口
3. 在文件末尾添加 `GNC_REGISTER_COMPONENT(类名, 接口名)`
4. 不需要修改 `main.cpp` 或 `CMakeLists.txt`
5. 放好文件后重新构建即可

## 模板

参见 `templates/` 目录。
