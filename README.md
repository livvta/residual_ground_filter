# Residual Ground Filter

此程序用于在地面分割算法之后，再次去除没有被正确分割的地面点。

地面高度带外的点无条件保留；带内二维稀疏点只有在邻居不呈明显竖直分布时才会删除，从而保护墙边、立柱和空中钢管等结构。

<table width="80%">
<tr>
  <td width="100%" align="center">
    <img src="img/filter_1.webp" width="100%" alt="残留地面点过滤效果">
  </td>
</tr>
</table>

## 算法流程

```text
PointCloud2
│
├─ 校验消息布局和 FLOAT32 x/y/z 字段
├─ 可选：变换到 input_frame
├─ 转换为 XYZ 点云并清理无效点
│  ├─ NaN / Inf：删除
│  ├─ (0,0,0)：按 remove_zero_points 配置删除
│  └─ z > pre_filter.max_z：直接保留，但不参与后续计算
│
├─ 其余有效点投影到 XY 平面
├─ 建立二维连续均匀网格
└─ 逐点分类
   ├─ 不在 deletion_z 高度带内：直接保留
   └─ 位于 deletion_z 高度带内：执行精确圆形邻域搜索
      ├─ 邻居数 ≥ min_neighbors：保留
      └─ 邻居数 < min_neighbors
         ├─ 满足竖直分布救回条件：保留
         └─ 不满足：作为残留地面点删除

保留点按输入顺序合并
└─ 可选：变换到 output_frame 后发布
```


`pre_filter.max_z` 用于隔离天花板等高处点。高于阈值的点会保留在输出中，但不会进入 XY 网格，也不会增加地面候选点的邻居数量。

## 编译

```bash
colcon build --packages-select residual_ground_filter --symlink-install
source install/setup.bash
```

## 运行

使用默认话题：

```bash
ros2 launch residual_ground_filter residual_ground_filter.launch.py
```

指定输入、输出话题：

```bash
ros2 launch residual_ground_filter residual_ground_filter.launch.py \
  input_topic:=/multi_lidar_ground_segmentation/nonground \
  output_topic:=/multi_lidar_ground_segmentation/nonground_filtered
```

也可以直接运行节点：

```bash
ros2 run residual_ground_filter residual_ground_filter_node \
  --ros-args \
  --params-file /work/ros2_humble/lidar_fusion_ws/src/residual_ground_filter/config/residual_ground_filter.param.yaml \
  -r input:=/multi_lidar_ground_segmentation/nonground \
  -r output:=/multi_lidar_ground_segmentation/nonground_filtered
```

## 参数

下表中的值与仓库内标准配置文件一致。

| 参数 | 标准配置值 | 说明 |
|---|---:|---|
| `search_radius` | `0.25` | XY 平面的精确圆形搜索半径（m） |
| `min_neighbors` | `22` | 半径内判定为稠密点所需的最少点数，包含查询点 |
| `remove_zero_points` | `true` | 是否删除 `(0,0,0)` 占位点 |
| `pre_filter.max_z` | `3.55` | 高于此 Z 值的点直接保留，不参与邻域计算（m） |
| `deletion_z.enabled` | `true` | 是否仅允许删除指定 Z 高度带内的点 |
| `deletion_z.min` | `-3.0` | 可删除高度带的最低 Z，包含边界（m） |
| `deletion_z.max` | `1.0` | 可删除高度带的最高 Z，包含边界（m） |
| `vertical_rescue.enabled` | `true` | 是否对稀疏点执行竖直分布保护 |
| `vertical_rescue.min_z_span` | `0.25` | 邻域所需的最小 Z 总跨度（m） |
| `vertical_rescue.separation_threshold` | `0.10` | 邻居与查询点达到此高度差时计为分离点（m） |
| `vertical_rescue.min_separated_points` | `4` | 所需的最少高度分离点数 |
| `vertical_rescue.z_window` | `2.00` | 查询点上下参与 Z 分箱的范围（m） |
| `vertical_rescue.bin_size` | `0.10` | Z 分箱高度（m） |
| `vertical_rescue.min_occupied_bins` | `3` | 所需的最少占用高度层数 |
| `input_frame` | 空 | 非空时先变换到该坐标系中处理 |
| `output_frame` | 空 | 非空时将结果变换到该坐标系；空值恢复输入坐标系 |
| `transform_timeout_sec` | `0.1` | TF 查询超时（s） |
| `max_queue_size` | `1` | 输入和输出的 SensorData QoS 队列深度 |

竖直分布保护仅在二维邻居数小于 `min_neighbors` 时执行。邻域需要同时满足最小 Z 跨度、最少高度分离点数和最少占用高度层数，才会被判定为竖直结构并保留。

## 运行时调参

以下算法参数支持 ROS 2 运行时更新：

- `search_radius`
- `min_neighbors`
- `remove_zero_points`
- `pre_filter.max_z`
- `deletion_z.*`
- `vertical_rescue.*`

坐标系、TF 超时和 QoS 队列属于节点结构参数，修改 `input_frame`、`output_frame`、`transform_timeout_sec` 或 `max_queue_size` 后需要重启节点。


## 调参建议

- 残留地面点仍然较多：适当增大 `min_neighbors`，或减小 `search_radius`。
- 远处稀疏目标被误删：适当减小 `min_neighbors`，或增大 `search_radius`。
- 墙边、立柱被误删：检查 `vertical_rescue.*`，优先降低 `min_separated_points` 或 `min_occupied_bins`。
- 天花板影响地面判断：降低 `pre_filter.max_z`，确认阈值以上点仍正常出现在输出中。
- 只希望处理车辆附近的地面高度带：收紧 `deletion_z.min` 和 `deletion_z.max`。
