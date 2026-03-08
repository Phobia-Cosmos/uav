# Archive (历史代码)

归档的历史代码和实验性代码，仅供参考，不建议在生产环境中使用。

## 文件说明

| 文件 | 功能描述 | 状态 |
| --- | --- | --- |
| `distance.py` | 超声波测距模块的独立测试版本。 | 已归档 |
| `cv2_test.py` | OpenCV 红色目标识别基础测试。 | 已归档 |
| `lati_hold_fly.py` | 早期手动 RC override 飞行实验脚本。 | 已归档 |
| `example1.py` | 早期 DroneKit 示例代码。 | 已归档 |

## 代码来源

这些文件来自项目的不同开发阶段：

1. **早期探索阶段** - `example1.py`
2. **传感器测试阶段** - `distance.py`, `cv2_test.py`
3. **功能迭代阶段** - `lati_hold_fly.py`

## 与当前代码的对比

| 功能 | 归档代码 | 当前代码 |
| --- | --- | --- |
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

## 本次清理

- 删除了 `a.py`：与 `lati_hold_fly.py` 内容高度重复，且命名无语义。
- 删除了 `data/simulate.py`：仓库内已明确标注为暂时不需要的旧本地模拟代码。
- 如果后续确实需要恢复，可直接从 Git 历史中找回。

## 如何恢复使用

```bash
# 超声波测试
python3 archive/distance.py

# 摄像头测试
python3 archive/cv2_test.py
```
