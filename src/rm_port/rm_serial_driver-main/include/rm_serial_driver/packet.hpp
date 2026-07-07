#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>
#include <array>

namespace rm_serial_driver
{
struct ReceivePacketA
{
  uint8_t header = 0xA5;
  uint8_t game_progress;         // 当前比赛状态
  uint16_t stage_remain_time;    // 比赛剩余时间
  uint8_t red_blue;              // 红蓝方（红方为1,蓝方为0）
             
  uint16_t self_1_hp;            //己方机器人血量
  uint16_t self_2_hp;
  uint16_t self_3_hp;
  uint16_t self_4_hp;
  uint16_t self_7_hp;
  uint16_t self_outpost_hp;      //己方前哨站血量
  uint16_t self_base_hp;         //己方基地血量
  uint16_t bullets_allowance;    //允许发弹量
  float chassis_power;           //chassis_power
  uint8_t self_small_energy;     //己方小能量机关激活状态
  uint8_t self_big_energy;       //己方大能量机关激活状态
  uint16_t total_bullets;        //哨兵累积发弹量

  uint16_t post_1_hp;            //敌方机器人血量
  uint16_t post_2_hp;
  uint16_t post_3_hp;
  uint16_t post_4_hp;
  uint16_t post_7_hp;
  uint16_t post_outpost_hp;      //敌方前哨站血量
  uint16_t post_base_hp;         //敌方基地血量
  uint16_t dart_time;            //对方飞镖最后一次击中己方前哨站或基地的时间
  uint8_t dart_type;             //对方飞镖最后一次击中己方前哨站或基地的具体目标

  uint16_t remain_gold;          //剩余金币
  uint8_t heat_flag;             //20000J热量是否用完
  float operator_x;              //操作手标点
  float operator_y;

  uint8_t decision_code;         //决策代号

  uint16_t checksum = 0;
} __attribute__((packed));

struct ReceivePacketUL
{
  uint8_t header = 0xA4;
  uint8_t game_progress;         // 当前比赛状态
  uint16_t stage_remain_time;    // 比赛剩余时间
  uint8_t red_blue;              // 红蓝方（红方为1,蓝方为0）
             
  uint16_t self_hero_hp;            //英雄血量
  uint16_t self_infantry_hp;        //步兵血量
  uint16_t self_sentry_hp;          //自身血量

  float self_pose_x;                //自身位置
  float self_pose_y;

  uint16_t bullets_allowance;       //允许发弹量

  float hero_pose_x;                //英雄位置
  float hero_pose_y;

  float infantry_pose_x;            //步兵位置
  float infantry_pose_y;

  uint8_t center_status;              //中心增益点占领情况
  uint16_t remain_gold;               //剩余金币

  uint16_t checksum = 0;
} __attribute__((packed));

struct ReceivePacketUC
{
  uint8_t header = 0xA3;
  uint8_t game_progress;         // 当前比赛状态
  uint16_t stage_remain_time;    // 比赛剩余时间
  uint8_t red_blue;              // 红蓝方（红方为1,蓝方为0）
             
  uint16_t self_hero_hp;            //英雄血量
  uint16_t self_engineer_hp;        //工程血量
  uint16_t self_infantry3_hp;        //步兵血量
  uint16_t self_infantry4_hp;        //步兵血量
  uint16_t self_sentry_hp;          //自身血量
  uint16_t self_outpost_hp;      //己方前哨站血量
  uint16_t self_base_hp;         //己方基地血量

  float self_pose_x;                //自身位置
  float self_pose_y;

  uint16_t bullets_allowance;       //允许发弹量

  float self_hero_pose_x;                //英雄位置
  float self_hero_pose_y;

  float self_engineer_pose_x;          //工程位置
  float self_engineer_pose_y;
  float self_infantry3_pose_x;            //步兵位置
  float self_infantry3_pose_y;
  float self_infantry4_pose_x;            //步兵位置
  float self_infantry4_pose_y;

  uint16_t remain_gold;               //剩余金币
  uint16_t recovery_buff;          //回血增益
  uint16_t defence_buff;            //防御增益
  uint16_t defence_debuff;          //防御debuff
  uint16_t attack_buff;             //攻击增益
  uint8_t status_info;              //最后3bit： 2bit占堡垒情况 + 最后1bit：是否中基地镖
  float self_speed_x;                //自身速度
  float self_speed_y;
  float operator_x;              //操作手标点
  float operator_y;
  uint8_t decision_node;           
  uint16_t checksum = 0;
} __attribute__((packed));

struct ReceivePacketB
{
  uint8_t header = 0xA6;
  // 雷达交互数据
  std::array<std::array<uint16_t, 2>, 5> enemy_poses;       //敌方位置x、y
  std::array<std::array<int16_t, 2>, 5> enemy_speed_xy;     //敌方XY轴速度
    
  uint16_t checksum = 0;
} __attribute__((packed));

struct SendPacket
{
  uint8_t header = 0xBE;
  float speed_x;
  float speed_y;
  float angle;                                          //转头角度 
  float pitch;                                          //抬头角度
  uint8_t shoot_mode;                                   //0为自瞄，1为打符，2为打前哨站
  uint8_t area_status;                               //特殊地形区域状态 1起伏路段 2前哨站 3下台阶 0默认 
  uint8_t nav_enable;                                   //默认0为不导航，发1为导航
  uint8_t sentry_stance;                                //1 为进攻姿态，2 为防御姿态，3 为移动姿态，默认为 3                                  
  std::array<std::array<float, 2>, 5> send_enemy_poses; //发给雷达补盲的消息(顺序：英雄、工程、步兵1、步兵2、哨兵)
  uint16_t checksum = 0;
} __attribute__((packed));

inline ReceivePacketA fromVectorA(const std::vector<uint8_t> & data)
{
  ReceivePacketA packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}
inline ReceivePacketUL fromVectorul(const std::vector<uint8_t> & data)
{
  ReceivePacketUL packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}
inline ReceivePacketUC fromVectoruc(const std::vector<uint8_t> & data)
{
  ReceivePacketUC packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}
inline ReceivePacketB fromVectorB(const std::vector<uint8_t> & data)
{
  ReceivePacketB packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

inline std::vector<uint8_t> toVector(const SendPacket & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
  return packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
