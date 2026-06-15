# 如何添加新的灵巧手模型 | How to Add New Dexterous Hand

## 🎯 核心答案

**YES，框架可以适配任意灵巧手，但需要：**
1. ✅ 机械手的 URDF 文件
2. ✅ 编写 YAML 配置文件
3. ✅ （可选）在常量文件中注册

**不需要修改源代码的任何优化器！** ✨

---

## 📋 适配新手的完整步骤

### 步骤1：准备URDF文件

URDF（Unified Robot Description Format）是定义机械手结构的XML格式文件。

#### 1.1 URDF应该包含的内容

```xml
<!-- 你的新手 URDF 文件: my_new_hand.urdf -->
<robot name="my_new_hand_right">
  <!-- 定义所有关节 -->
  <joint name="wrist" type="fixed">
    <!-- ... -->
  </joint>
  
  <joint name="finger1_q1" type="revolute">
    <limit lower="-3.14" upper="3.14" effort="1000" velocity="100"/>
    <parent link="palm"/>
    <child link="finger1_link1"/>
  </joint>
  
  <!-- 定义所有链接 -->
  <link name="palm"/>
  <link name="finger1_link1"/>
  
  <!-- 定义运动学树 -->
  <!-- ... -->
</robot>
```

#### 1.2 URDF需要满足的条件

```python
# 来自 dex_retargeting/robot_wrapper.py 第 13-23 行

class RobotWrapper:
    def __init__(self, urdf_path: str):
        # Pinocchio库加载URDF
        self.model = pin.buildModelFromUrdf(urdf_path)
        
        # ✅ 要求1：所有关节的DoF必须 <= 2
        if self.model.nv != self.model.nq:
            raise NotImplementedError(f"不能处理特殊关节")
        
        # ✅ 要求2：关节不能有约束（如可变质量）
        
        # ✅ 要求3：URDF格式必须正确
```

#### 1.3 常见URDF来源

```
┌─────────────────────────────────────────────────┐
│ 获取灵巧手URDF的方式                            │
├─────────────────────────────────────────────────┤
│ 1. 官方GitHub仓库                               │
│    ├─ Allegro Hand: https://github.com/HIRO-group/allegro_hand
│    ├─ Shadow Hand: https://github.com/shadow-robot/shadow_robot
│    └─ ...其他手的GitHub                         │
│                                                 │
│ 2. 使用URDF创建工具                             │
│    └─ 如果没有URDF，需要手动创建或转换          │
│                                                 │
│ 3. 从其他格式转换                               │
│    └─ SDF/STEP/STL → URDF (需要MuJoCo/Drake工具)
│                                                 │
│ 4. CAD工具导出                                  │
│    └─ 从ROS/Gazebo插件导出                      │
└─────────────────────────────────────────────────┘
```

---

### 步骤2：创建YAML配置文件

这是最核心的适配工作！

#### 2.1 YAML配置文件应该放在哪里？

```
dex_retargeting/configs/
├── teleop/                    # ← 实时遥操作配置
│   ├── allegro_hand_right.yml
│   ├── shadow_hand_right.yml
│   └── my_new_hand_right.yml  # ← 你的新配置文件！
│
└── offline/                   # ← 离线处理配置
    └── my_new_hand_right.yml
```

#### 2.2 配置文件的关键内容

```yaml
# dex_retargeting/configs/teleop/my_new_hand_right.yml

retargeting:
  type: vector  # 或 "position" 或 "dexpilot"
  urdf_path: path/to/my_new_hand_right.urdf
  
  # ============ 最关键的部分 ============
  # 定义哪些人手关键点用来跟踪机械手
  
  # 向量的起点：通常是基部（手腕或手掌）
  target_origin_link_names: 
    - "palm"        # 向量1的起点
    - "palm"        # 向量2的起点
    - "palm"        # 向量3的起点
    - "palm"        # 向量4的起点
  
  # 向量的终点：通常是指尖
  target_task_link_names: 
    - "finger1_tip"   # 向量1的终点
    - "finger2_tip"   # 向量2的终点
    - "finger3_tip"   # 向量3的终点
    - "finger4_tip"   # 向量4的终点
  
  # 人手关键点的对应关系
  # 这定义了"用哪些人手点来生成向量"
  target_link_human_indices: 
    - [ 0, 0, 0, 0 ]           # 4个向量的起点：都是点0（手腕）
    - [ 4, 8, 12, 16 ]         # 4个向量的终点：4个指尖
  
  # 人手和机械手的大小比例
  # 例如：Allegro手比人手小1.6倍
  scaling_factor: 1.6
  
  # 其他参数
  low_pass_alpha: 0.2      # 低通滤波器（0.1~0.3）
  has_joint_limits: true   # 使用关节角度范围约束
```

#### 2.3 配置文件的三种类型

##### **类型1：Vector Retargeting（最常用）**

```yaml
retargeting:
  type: vector
  urdf_path: hands/my_hand/my_hand_right.urdf
  
  # 生成向量（起点和终点）
  target_origin_link_names: ["palm", "palm", "palm", "palm"]
  target_task_link_names: ["tip1", "tip2", "tip3", "tip4"]
  
  target_link_human_indices: 
    - [0, 0, 0, 0]           # 向量起点：点0（手腕）
    - [4, 8, 12, 16]         # 向量终点：点4,8,12,16（4个指尖）
  
  scaling_factor: 1.6
  low_pass_alpha: 0.2

# 适合：
# ✅ 遥操作系统（不关心绝对位置）
# ✅ 大部分灵巧手
```

##### **类型2：Position Retargeting（位置匹配）**

```yaml
retargeting:
  type: position
  urdf_path: hands/my_hand/my_hand_right.urdf
  
  # 直接跟踪链接的3D位置（不是向量）
  target_link_names: ["tip1", "tip2", "tip3", "tip4"]
  
  target_link_human_indices: 
    - [4, 8, 12, 16]         # 链接0跟踪人手点4，链接1跟踪点8，...
  
  scaling_factor: 1.6
  low_pass_alpha: 0.2

# 适合：
# ✅ 手-物体交互任务
# ✅ 需要准确的绝对位置
```

##### **类型3：DexPilot Retargeting（智能抓握）**

```yaml
retargeting:
  type: dexpilot
  urdf_path: hands/my_hand/my_hand_right.urdf
  
  # 指尖和腕部
  wrist_link_name: "palm"
  finger_tip_link_names: ["tip1", "tip2", "tip3", "tip4"]
  
  # 可选：自定义目标点
  target_link_human_indices: null  # 使用默认值
  
  # DexPilot特定参数
  project_dist: 0.03    # 手指接近距离阈值（3cm）
  escape_dist: 0.05     # 手指分开距离阈值（5cm）
  
  scaling_factor: 1.6
  low_pass_alpha: 0.2

# 适合：
# ✅ 需要自动"抓握"的任务
# ✅ 手指需要接触约束的场景
```

---

### 步骤3：确定链接和关节名称

最容易出错的地方！

#### 3.1 如何找到URDF中的链接名称？

```python
# 临时脚本：check_urdf.py

from dex_retargeting.robot_wrapper import RobotWrapper

# 加载你的URDF
robot = RobotWrapper("path/to/my_new_hand.urdf")

# 打印所有关节
print("关节名称:")
for i, name in enumerate(robot.dof_joint_names):
    print(f"  {i}: {name}")

# 打印所有链接
print("\n链接名称:")
for i, name in enumerate(robot.link_names):
    print(f"  {i}: {name}")

# 打印关节角度范围
print("\n关节角度范围:")
for i, name in enumerate(robot.dof_joint_names):
    limits = robot.joint_limits[i]
    print(f"  {name}: [{limits[0]:.2f}, {limits[1]:.2f}]")
```

运行这个脚本输出：

```
关节名称:
  0: thumb_roll
  1: thumb_pip
  2: thumb_dip
  3: index_mcp
  4: index_pip
  5: index_dip
  6: middle_mcp
  ...

链接名称:
  0: palm
  1: thumb_link1
  2: thumb_link2
  3: thumb_link3
  4: thumb_tip
  5: index_link1
  ...

关节角度范围:
  thumb_roll: [-0.47, 0.47]
  thumb_pip: [0.00, 1.57]
  thumb_dip: [0.00, 1.57]
  ...
```

#### 3.2 关键概念

```
URDF中的结构：

base_link (通常是"手掌")
    |
    ├─ joint1 (回转关节)
    |   └─ link1
    |       |
    |       ├─ joint2
    |       |   └─ link2
    |       |
    |       └─ 其他链接...
    |
    ├─ 其他手指...
    |
    └─ ...

示例（Allegro Hand）：
palm
    |
    ├─ joint_0.0 (拇指)
    |   └─ link_0.0
    |       └─ joint_0.1
    |           └─ link_0.1
    |               └─ joint_0.2
    |                   └─ link_0.2 (拇指第二节)
    |                       └─ joint_0.3
    |                           └─ link_0.3_tip (拇指指尖) ← 这个最重要！
    |
    ├─ joint_1.0 (食指)
    |   └─ link_1.0
    |       └─ link_1.0_tip (食指指尖)
    |
    └─ ...
```

---

### 步骤4：优化关键参数

#### 4.1 scaling_factor（关键参数！）

```python
# 定义：灵巧手的尺寸 / 人手的尺寸

# 如何测量？
# 1. 测量人手长度：手腕到指尖，约20cm
# 2. 测量机械手长度：手腕到指尖，例如32cm
# 3. scaling_factor = 32 / 20 = 1.6

# 常见值：
allegro_hand:  1.6
shadow_hand:   1.2
leap_hand:     1.4
my_new_hand:   ?  # 需要自己计算！

# 验证方法：
# 如果缩放因子太大：机械手动作幅度太大
# 如果缩放因子太小：机械手无法充分展开
```

#### 4.2 low_pass_alpha（平滑参数）

```python
# 定义：低通滤波器的平滑程度 (0.0 ~ 1.0)

# low_pass_alpha = 0.0: 完全跟随（无平滑）
# low_pass_alpha = 0.5: 中等平滑
# low_pass_alpha = 1.0: 完全忽略新数据（无响应）

# 推荐值：0.1 ~ 0.3
#   - 0.1: 更平滑但延迟大
#   - 0.3: 更快速但可能抖动
```

#### 4.3 选择合适的向量

```python
# Vector Retargeting需要选择有代表性的向量

# 差的选择（可能导致映射失败）：
target_origin_link_names: ["tip1", "tip2", "tip3", "tip4"]
target_task_link_names:   ["tip1", "tip2", "tip3", "tip4"]
# ❌ 起点和终点相同！无法生成向量

# 好的选择：
target_origin_link_names: ["palm", "palm", "palm", "palm"]
target_task_link_names:   ["tip1", "tip2", "tip3", "tip4"]
# ✅ 表示"每个指尖相对于手掌"的向量

# 另一个好的选择：
target_origin_link_names: ["link_1", "link_1", "link_1", "link_1"]
target_task_link_names:   ["tip1", "tip2", "tip3", "tip4"]
# ✅ 表示"每个指尖相对于中心关节"的向量
```

---

### 步骤5：注册机械手（可选）

如果你想在常数中注册新手，方便调用：

#### 5.1 修改 constants.py

```python
# dex_retargeting/constants.py

class RobotName(enum.Enum):
    allegro = enum.auto()
    shadow = enum.auto()
    my_new_hand = enum.auto()  # ← 添加新手

ROBOT_NAME_MAP = {
    RobotName.allegro: "allegro_hand",
    RobotName.shadow: "shadow_hand",
    RobotName.my_new_hand: "my_new_hand",  # ← 添加映射
}
```

#### 5.2 现在就可以这样使用

```python
from dex_retargeting.constants import RobotName, RetargetingType, HandType

# 使用新手
python wilor_mini/teleoperate.py \
    --robot-name my_new_hand \
    --retargeting-type vector \
    --hand-type right
```

---

## 🔍 完整适配检查清单

```
┌─────────────────────────────────────────────────┐
│ 新手适配检查清单                                │
├─────────────────────────────────────────────────┤
│                                                 │
│ □ 1. 获得URDF文件                              │
│    └─ 路径: assets/robots/hands/my_hand/      │
│                                                 │
│ □ 2. 验证URDF格式                              │
│    └─ 运行: check_urdf.py                     │
│    └─ 确保所有关节都被识别                     │
│                                                 │
│ □ 3. 记录链接名称                              │
│    └─ palm, tip1, tip2, ...                    │
│                                                 │
│ □ 4. 创建YAML配置文件                          │
│    └─ 复制相似手的config作为模板              │
│    └─ 修改路径、链接名称、点索引              │
│                                                 │
│ □ 5. 测试配置                                  │
│    └─ 运行: python wilor_mini/teleoperate.py  │
│    └─ 检查是否能正确控制                       │
│                                                 │
│ □ 6. 调整参数                                  │
│    └─ 微调 scaling_factor                     │
│    └─ 微调 low_pass_alpha                     │
│    └─ 微调 向量选择                            │
│                                                 │
│ □ 7. 验证运动学                                │
│    └─ 检查关节是否合理移动                     │
│    └─ 检查是否有奇异点问题                     │
│                                                 │
└─────────────────────────────────────────────────┘
```

---

## 🎓 实际例子：添加新手"MyHand"

### 假设你有一个新的灵巧手"MyHand"：

**步骤1：放置URDF文件**

```bash
assets/robots/hands/
├── my_hand/
│   ├── my_hand_right.urdf    ← URDF文件
│   ├── my_hand_right_glb.urdf
│   └── meshes/               ← 可视化模型
│       ├── palm.stl
│       ├── finger1.stl
│       └── ...
```

**步骤2：创建YAML配置**

```yaml
# dex_retargeting/configs/teleop/my_hand_right.yml

retargeting:
  type: vector
  urdf_path: my_hand/my_hand_right.urdf
  
  target_origin_link_names: ["palm", "palm", "palm", "palm"]
  target_task_link_names: ["finger1_tip", "finger2_tip", "finger3_tip", "finger4_tip"]
  
  target_link_human_indices:
    - [0, 0, 0, 0]
    - [4, 8, 12, 16]
  
  scaling_factor: 1.5
  low_pass_alpha: 0.2
```

**步骤3：测试**

```bash
python wilor_mini/teleoperate.py \
    --robot-name allegro \  # 临时用allegro的配置测试
    --retargeting-type vector \
    --hand-type right
```

不行？改成你的URDF路径再试...

**步骤4：优化参数**

```python
# 运行 check_urdf.py 查看实际的链接名称
# 调整 scaling_factor 直到动作幅度合适
```

---

## ⚠️ 常见错误

| 错误 | 原因 | 解决方案 |
|------|------|--------|
| `RuntimeError: Can not handle robot with special joint` | URDF中有特殊关节（DoF > 2） | 检查URDF，移除或简化特殊关节 |
| `ValueError: {name} is not a link name` | YAML中的链接名称不存在 | 运行check_urdf.py确认正确的名称 |
| `机械手动作太大/太小` | scaling_factor不对 | 调整scaling_factor值 |
| `优化无法收敛` | 向量选择不合理 | 改变target_origin/task_link_names |
| `机械手抖动` | low_pass_alpha太大 | 减小low_pass_alpha（如0.1） |

---

## 📊 框架支持矩阵

```
灵巧手类型          支持   说明
─────────────────────────────────────────────
Allegro Hand (4F)   ✅    完全支持（内置）
Shadow Hand (5F)    ✅    完全支持（内置）
Leap Hand (4F)      ✅    完全支持（内置）
Panda Gripper (2F)  ✅    完全支持（内置）
SVH Hand (5F)       ✅    完全支持（内置）
Inspire Hand (5F)   ✅    完全支持（内置）

任意灵巧手          ✅    支持！需要：
  - URDF文件              1. URDF文件
  - 任意自由度            2. YAML配置文件
  - 任意指数              3. 可选：注册在常数

─────────────────────────────────────────────
```

---

## 🚀 总结

| 方面 | 答案 |
|------|------|
| **能否适配其他手？** | ✅ 能，框架通用 |
| **是否需要修改源代码？** | ❌ 不需要 |
| **需要什么？** | 1. URDF 2. YAML配置 3. (可选)注册常数 |
| **难度级别？** | 🟡 中等（配置不复杂，调参需要时间） |
| **时间成本？** | ⏱️ 1-2小时（获取URDF + 配置 + 调参） |

**最重要的是：配置文件和URDF！** 🎯

