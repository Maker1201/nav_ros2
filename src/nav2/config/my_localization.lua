-- 直接包含我们之前写好的、已经调通的建图配置
include "my_cartographer_2d.lua"

-- ==========================================
-- 纯定位模式专属配置
-- ==========================================

-- 告诉 Cartographer：不要再无限扩大地图了，内存里只保留最新的 3 个子图用来跟静态地图匹配即可
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

-- 加快全局优化的频率（因为不用建图了，算力可以全部用来高频定位）
POSE_GRAPH.optimize_every_n_nodes = 20

return options