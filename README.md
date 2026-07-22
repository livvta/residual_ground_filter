# Radius Search 2D Outlier Filter

程序将输入转换为 `pcl::PointCloud<pcl::PointXYZ>`，把 Z 置零建立二维均匀网格。
网格边长等于 `search_radius`；每次只检查当前格及周围 8 格，再执行精确圆形距离判断。

用于在地面分割算法之后，再次去除没有被正确分割的地面点。地面高度带外的点无条件
保留；带内二维稀疏点只有在邻居不呈明显竖直分布时才会删除，从而保护墙边、立柱和
空中钢管等结构。

## 处理逻辑

```

  PointCloud2
  │
  ├─ 检查消息结构
  │  ├─ 没有 FLOAT32 x/y/z → 丢弃整帧
  │  └─ 消息结构正常
  │
  ├─ 可选 TF 变换到 input_frame
  │
  ├─ 转换为 PCL XYZ 点云
  │
  ├─ 逐点预清理
  │  ├─ x/y/z 含 NaN 或 Inf → 删除
  │  ├─ remove_zero_points=true 且为 (0,0,0) → 删除
  │  └─ 有效点 → 保留到待处理集合
  │
  ├─ 将所有有效点投影到 XY 平面
  │  ├─ 投影点的 Z 统一设为 0
  │  └─ 原始 Z 单独保存在对齐数组中
  │
  ├─ 建立二维连续均匀网格
  │
  └─ 对每个有效点执行判断
     │
     ├─ 查询点 Z 是否位于允许删除的高度带？
     │  │
     │  │  当前范围：[-2.50, -1.60] m
     │  │
     │  ├─ 否 → 无条件保留
     │  │        └─ 不执行二维邻域搜索
     │  │
     │  └─ 是 → 执行二维半径搜索
     │           │
     │           │  设置search_radius参数
     │           │
     │           ├─ 邻居数 ≥ min_neighbors
     │           │  └─ 保留
     │           │
     │           └─ 邻居数 < min_neighbors
     │              │
     │              ├─ vertical_rescue=false
     │              │  └─ 删除
     │              │
     │              └─ vertical_rescue=true
     │                 │
     │                 ├─ 检查邻居 Z 总跨度
     │                 │  └─ 是否 ≥ min_z_span
     │                 │
     │                 ├─ 检查高度分离点
     │                 │  ├─ |邻居Z - 查询点Z| ≥ separation_threshold
     │                 │  │  → 计为一个分离点
     │                 │  └─ 是否至少有 min_separated_points 个分离点？
     │                 │
     │                 ├─ 检查 Z 高度层分布
     │                 │  ├─ 查询点上下各 z_window 高度区间
     │                 │  ├─ 每 bin_size 一个分箱（高度层）
     │                 │  └─ 是否至少占据 min_occupied_bins 个高度层？
     │                 │
     │                 ├─ 三个条件全部满足
     │                 │  └─ 判定为竖直结构 → 保留
     │                 │
     │                 └─ 任意条件不满足
     │                    └─ 判定为稀疏非竖直点 → 删除
     │
     ├─ 所有保留点转换为 XYZ PointCloud2
     │
     ├─ 保持输入时间戳
     │
     ├─ 可选 TF 变换到 output_frame
     │
     └─ 输出：/patchworkpp/nonground_filtered

```


## 与 Autoware 的关系

- 保留二维精确圆形半径判据、参数名称和 `input`/`output` 话题接口；邻域索引改为连续均匀
  网格，不再使用 PCL KD-tree。
- 保留 SensorData QoS、运行时参数更新、可组合组件、独立可执行程序和可选 TF 变换。
- 没有引入 Autoware 整包中的诊断、调试发布器、managed transform buffer 和 indices
  同步，因为这些会带入大量与本过滤器无关的 Autoware 依赖。
- `remove_zero_points` 是本项目扩展。配置中默认开启，以清除雷达驱动或上游处理中产生的
  `(0,0,0)` 占位点；设为 `false` 时，核心点选择判据与原实现一致。

## 编译

```bash
cd /work/ros2_humble/lidar_fusion_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select radius_search_2d_outlier_filter --symlink-install
source install/setup.bash
```

## 运行

默认连接 `/patchworkpp/nonground`：

```bash
ros2 launch radius_search_2d_outlier_filter radius_search_2d_outlier_filter.launch.py
```

修改话题：

```bash
ros2 launch radius_search_2d_outlier_filter radius_search_2d_outlier_filter.launch.py \
  input_topic:=/patchworkpp/nonground \
  output_topic:=/patchworkpp/nonground_filtered
```

也可以直接运行节点：

```bash
ros2 run radius_search_2d_outlier_filter radius_search_2d_outlier_filter_node \
  --ros-args \
  --params-file /work/ros2_humble/lidar_fusion_ws/src/radius_search_2d_outlier_filter/config/radius_search_2d_outlier_filter.param.yaml \
  -r input:=/patchworkpp/nonground \
  -r output:=/patchworkpp/nonground_filtered
```

## 参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `search_radius` | `0.2` | XY 平面搜索半径（m） |
| `min_neighbors` | `5` | 半径内最少点数，包含点自身 |
| `remove_zero_points` | `true`（配置文件） | 删除 `(0,0,0)` 占位点 |
| `deletion_z.enabled` | `true` | 只允许删除指定 Z 高度带内的点 |
| `deletion_z.min` | `-2.50` | 允许删除的最低 Z（m，包含边界） |
| `deletion_z.max` | `-1.60` | 允许删除的最高 Z（m，包含边界） |
| `vertical_rescue.enabled` | `true` | 对原本将删除的稀疏点启用 Z 分布判定 |
| `vertical_rescue.min_z_span` | `0.25` | 邻域最小 Z 跨度（m） |
| `vertical_rescue.separation_threshold` | `0.10` | 与查询点明显分离的高度差（m） |
| `vertical_rescue.min_separated_points` | `3` | 最少高度分离邻居数 |
| `vertical_rescue.bin_size` | `0.10` | Z 分箱尺寸（m） |
| `vertical_rescue.z_window` | `0.80` | 查询点上下统计范围（m） |
| `vertical_rescue.min_occupied_bins` | `3` | 最少占用高度层数 |
| `input_frame` | 空 | 非空时先变换到该坐标系处理 |
| `output_frame` | 空 | 非空时将结果变换到该坐标系；空则恢复输入 frame |
| `transform_timeout_sec` | `0.1` | TF 查询超时（s） |
| `max_queue_size` | `5` | SensorData QoS 队列深度 |

`search_radius`、`min_neighbors`、`remove_zero_points`、全部 `deletion_z.*` 和
`vertical_rescue.*` 参数可通过 `ros2 param set` 或 rqt 动态修改。TF 与队列参数需要
重启节点。

`deletion_z` 判断发生在二维邻域搜索之前。高度带外的点直接复制到输出，不执行邻域
搜索；高度边界使用过滤器处理坐标系，当前 `/patchworkpp/nonground` 为 `base_link`。

竖直救回只在邻域点数量小于 `min_neighbors` 时运行，并复用本次邻域搜索
结果，不会执行第二次邻域查询。Z 数组与过滤后的二维点云索引严格对齐，判定函数
不进行逐点动态内存分配。

均匀网格使用连续桶存储点索引，并将最大返回数量限制为 `min_neighbors`。过滤判据只关心
邻居数量是否达到阈值，因此找到足够邻居后会立即停止；邻居不足时仍返回精确圆内的全部
邻居供 Z 分布判断使用，不改变原有分类判据。

## 调参方向

- 噪点仍太多：增大 `search_radius` 或 `min_neighbors`。
- 细杆、远处目标被删：减小 `min_neighbors`，其次适当增大 `search_radius`。
- 点密度随距离下降明显时，固定半径算法通常需要在“近处去噪”和“远处保留”之间折中。
