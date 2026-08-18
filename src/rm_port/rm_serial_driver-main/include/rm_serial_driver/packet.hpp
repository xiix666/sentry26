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
  uint8_t game_progress;
  uint16_t stage_remain_time;
  uint8_t red_blue;
             
  uint16_t self_1_hp;
  uint16_t self_2_hp;
  uint16_t self_3_hp;
  uint16_t self_4_hp;
  uint16_t self_7_hp;
  uint16_t self_outpost_hp;
  uint16_t self_base_hp;
  uint16_t bullets_allowance;
  float chassis_power;
  uint8_t self_small_energy;
  uint8_t self_big_energy;
  uint16_t total_bullets;

  uint16_t post_1_hp;
  uint16_t post_2_hp;
  uint16_t post_3_hp;
  uint16_t post_4_hp;
  uint16_t post_7_hp;
  uint16_t post_outpost_hp;
  uint16_t post_base_hp;
  uint16_t dart_time;
  uint8_t dart_type;

  uint16_t remain_gold;
  uint8_t heat_flag;
  float operator_x;
  float operator_y;

  uint8_t decision_code;

  uint16_t checksum = 0;
} __attribute__((packed));

struct ReceivePacketUL
{
  uint8_t header = 0xA4;
  uint8_t game_progress;
  uint16_t stage_remain_time;
  uint8_t red_blue;
             
  uint16_t self_hero_hp;
  uint16_t self_infantry_hp;
  uint16_t self_sentry_hp;

  float self_pose_x;
  float self_pose_y;

  uint16_t bullets_allowance;

  float hero_pose_x;
  float hero_pose_y;

  float infantry_pose_x;
  float infantry_pose_y;

  uint8_t center_status;
  uint16_t remain_gold;

  uint16_t checksum = 0;
} __attribute__((packed));

struct ReceivePacketUC
{
  uint8_t header = 0xA3;
  uint8_t game_progress;
  uint16_t stage_remain_time;
  uint8_t red_blue;
             
  uint16_t self_hero_hp;
  uint16_t self_engineer_hp;
  uint16_t self_infantry3_hp;
  uint16_t self_infantry4_hp;
  uint16_t self_sentry_hp;
  uint16_t self_outpost_hp;
  uint16_t self_base_hp;
  uint16_t enemy_outpost_hp;
  float self_pose_x;
  float self_pose_y;

  uint16_t bullets_allowance;

  float self_hero_pose_x;
  float self_hero_pose_y;

  float self_engineer_pose_x;
  float self_engineer_pose_y;
  float self_infantry3_pose_x;
  float self_infantry3_pose_y;
  float self_infantry4_pose_x;
  float self_infantry4_pose_y;

  uint16_t remain_gold;
  uint16_t recovery_buff;
  uint16_t defence_buff;
  uint16_t defence_debuff;
  uint16_t attack_buff;
  uint16_t status_info;
  float self_speed_x;
  float self_speed_y;
  float operator_x;
  float operator_y;
  uint8_t enemy_invincible;
  uint8_t decision_node;           
  uint16_t checksum = 0;
} __attribute__((packed));

struct ReceivePacketB
{
  uint8_t header = 0xA6;

  std::array<std::array<uint16_t, 2>, 5> enemy_poses;
  std::array<std::array<int16_t, 2>, 5> enemy_speed_xy;
    
  uint16_t checksum = 0;
} __attribute__((packed));

struct SendPacket
{
  uint8_t header = 0xBE;
  float speed_x;
  float speed_y;
  float angle;
  float pitch;
  uint8_t shoot_mode;
  uint8_t area_status;
  uint8_t nav_enable;
  uint8_t sentry_stance;
  uint8_t outpost_enable;
  uint8_t go_qifu;
  std::array<std::array<float, 2>, 5> send_enemy_poses;
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

}

#endif
