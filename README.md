# Radius Search 2D Outlier Filter

程序将输入转换为`pcl::PointCloud<pcl::PointXYZ>`，把 Z 置零建立二维 PCL KD-tree，然后保留搜索半径内
邻居数不少于阈值的原始 XYZ 点。

适合接在地面分割之后，用于清除雨点、飞虫和稀疏孤立噪点。不要直接用于包含完整地面的
原始点云，否则密集地面会改变邻域统计。

## 与 Autoware 的关系

- 保留 PCL KD-tree、二维半径搜索、参数名称和 `input`/`output` 话题接口。
- 保留 SensorData QoS、运行时参数更新、可组合组件、独立可执行程序和可选 TF 变换。
- 没有引入 Autoware 整包中的诊断、调试发布器、managed transform buffer 和 indices
  同步，因为这些会带入大量与本过滤器无关的 Autoware 依赖。
- `remove_zero_points` 是本项目扩展。配置中默认开启，以清除雷达驱动或上游处理中产生的
  `(0,0,0)` 占位点；设为 `false` 时，核心点选择过程与 Autoware 实现一致。

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
| `input_frame` | 空 | 非空时先变换到该坐标系处理 |
| `output_frame` | 空 | 非空时将结果变换到该坐标系；空则恢复输入 frame |
| `transform_timeout_sec` | `0.1` | TF 查询超时（s） |
| `max_queue_size` | `5` | SensorData QoS 队列深度 |

`search_radius`、`min_neighbors` 和 `remove_zero_points` 可通过
`ros2 param set` 或 rqt 动态修改。其余参数需要重启节点。

实现会将 PCL `radiusSearch` 的最大返回数量限制为 `min_neighbors`。过滤判据只关心
邻居数量是否达到阈值，因此该提前停止优化不会改变分类结果，并可显著减少密集区域中的
邻居枚举和临时内存使用。

## 调参方向

- 噪点仍太多：增大 `search_radius` 或 `min_neighbors`。
- 细杆、远处目标被删：减小 `min_neighbors`，其次适当增大 `search_radius`。
- 点密度随距离下降明显时，固定半径算法通常需要在“近处去噪”和“远处保留”之间折中。

