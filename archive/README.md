# Archive (历史代码)

归档的历史代码和实验性代码，仅供参考，不建议在生产环境中使用。

## 文件说明

| 文件 | 功能描述 | 状态 |
|-----|---------|------|
| `distance.py` | 超声波测距模块。独立测试代码，用于验证HC-SR04超声波模块。 | 已归档 |
| `cv2_test.py` | OpenCV摄像头测试。红色目标识别基础测试，包含HSV颜色空间分割。 | 已归档 |
| `lati_hold_fly.py` | 纬度保持飞行代码。历史版本的纬度保持飞行功能。 | 已归档 |
| `example1.py` | 示例代码。早期开发的示例代码，供参考学习。 | 已归档 |
| `a.py` | 旧代码。功能不明的历史代码，保留仅供参考。 | 已归档 |
| `data/simulate.py` | 模拟通信数据。暂时不需要，可丢弃。 | 待定 |

## 代码来源

这些文件来自项目的不同开发阶段：

1. **早期探索阶段** - `example1.py`, `a.py`
2. **传感器测试阶段** - `distance.py`, `cv2_test.py`
3. **功能迭代阶段** - `lati_hold_fly.py`

## 与当前代码的对比

| 功能 | 归档代码 | 当前代码 |
|-----|---------|---------|
| 超声波测距 | `archive/distance.py` | `src/drone/fly.py` (集成版) |
| 摄像头识别 | `archive/cv2_test.py` | `src/drone/fly.py` (集成版) |
| 飞控连接 | `archive/lati_hold_fly.py` | `src/drone/drone_server.py` |

## 是否可以使用？

**不建议直接使用**，原因：

1. 代码风格不一致
2. 缺少错误处理
3. 未经过充分测试
4. 可能与当前系统不兼容

**建议**：如需参考，请提取需要的函数/逻辑，集成到 `src/` 目录下的对应模块中。

## 如何恢复使用

```bash
# 超声波测试
python3 archive/distance.py

# 摄像头测试
python3 archive/cv2_test.py
```
