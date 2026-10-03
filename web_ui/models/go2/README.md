# Unitree Go2 可视化模型（建图 3D）

来源：[google-deepmind/mujoco_menagerie `unitree_go2`](https://github.com/google-deepmind/mujoco_menagerie/tree/main/unitree_go2)  
上游源自 [unitreerobotics/unitree_ros go2_description](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/go2_description)  
许可证：BSD-3-Clause（见 `LICENSE`）

| 文件 | 说明 |
|------|------|
| `go2_stand.bin` | 站立姿态烘焙网格（供网页加载） |
| `go2_stand.meta.json` | 元数据 |
| `bake_go2_stand.py` | 从 OBJ 重新烘焙 |

源 OBJ 在仓库 `third_party/unitree_go2/`（不通过 :8080 直接提供，避免 28MB 静态流量）。

重新烘焙：

```bash
# ASSETS 指向 third_party
python3 bake_go2_stand.py
```
