# 哨兵当前锁敌目标。
#   0 — 未锁敌
#   1 — 敌方英雄
#   2 — 敌方工程
#   3 — 敌方步兵（对应黑板槽位 infantry1）
#   4 — 敌方步兵（对应黑板槽位 infantry2）
#   5 — 无意义（保留，追击等逻辑视为无效目标）
#   6 — 前哨站（黑板 @enemy_outpost_*；固定坐标由 rm2026_decision_params.yaml 中 enemy_outpost_* 预置）
#   7 — 敌方哨兵
#   8 — 基地（黑板 @enemy_base_*；固定坐标由 rm2026_decision_params.yaml 中 enemy_base_* 预置）
int32 lock_enemy_id
