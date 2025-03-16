#pragma once

#include <OgreCamera.h>

#include <rviz_common/frame_position_tracking_view_controller.hpp>
#include <rviz_common/panel.hpp>

#include "devastator_perception_msgs/msg/image_radar_packet.hpp"
#include "radar_ars548_nodes/message/define.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "std_msgs/msg/header.hpp"

class QLabel;
class QLineEdit;
class QPushButton;
class QProgressBar;
class QCheckBox;
class QComboBox;
class SpeedBar;
class AccBar;
class TurnLightBar;

namespace radar_rviz_panel {

using PacketMsg = devastator_perception_msgs::msg::ImageRadarPacket;
using SensorStatusMsg = devastator::driver::SensorStatusMsg;

class RadarRvizPanel : public rviz_common::Panel {
  Q_OBJECT

 public:
  explicit RadarRvizPanel(QWidget* parent = 0);

  void SensorStatusChanged(const PacketMsg::SharedPtr msg);

  void SetLabeltStyle(QLabel* label, QFont font, int width, Qt::Alignment aalignment,
                      QString color);

 protected:
  void onInitialize() override;

 protected:
  QLabel* sec_l;
  QLineEdit* sec_e;

  QLabel* nano_l;
  QLineEdit* nano_e;

  QLabel* sync_l;
  QLineEdit* sync_e;

  QLabel* version_l;
  QLineEdit* version_e;

  QLabel* lon_position_l;
  QLineEdit* lon_position_e;

  QLabel* lat_position_l;
  QLineEdit* lat_position_e;

  QLabel* ver_position_l;
  QLineEdit* ver_position_e;

  QLabel* yaw_l;
  QLineEdit* yaw_e;

  QLabel* pitch_l;
  QLineEdit* pitch_e;

  QLabel* orientation_plug_l;
  QLineEdit* orientation_plug_e;

  QLabel* vehicle_length_l;
  QLineEdit* vehicle_length_e;

  QLabel* vehicle_width_l;
  QLineEdit* vehicle_width_e;

  QLabel* vehicle_height_l;
  QLineEdit* vehicle_height_e;

  QLabel* vehicle_wheelbase_l;
  QLineEdit* vehicle_wheelbase_e;

  QLabel* max_detec_distance_l;
  QLineEdit* max_detec_distance_e;

  QLabel* center_frequency_l;
  QLineEdit* center_frequency_e;

  QLabel* cycle_time_l;
  QLineEdit* cycle_time_e;

  QLabel* cycle_offest_l;
  QLineEdit* cycle_offest_e;

  QLabel* country_code_l;
  QLineEdit* country_code_e;

  QLabel* power_mode_l;
  QLineEdit* power_mode_e;

  QLabel* ip_l;
  QLineEdit* ip_e;

  QLabel* ip_reserve_l;
  QLineEdit* ip_reserve_e;

  QLabel* cfg_counter_l;
  QLineEdit* cfg_counter_e;

  QLabel* speed_input_l;
  QLineEdit* speed_input_e;

  QLabel* lon_acc_input_l;
  QLineEdit* lon_acc_input_e;

  QLabel* lat_acc_input_l;
  QLineEdit* lat_acc_input_e;

  QLabel* yawrate_input_l;
  QLineEdit* yawrate_input_e;

  QLabel* steer_input_l;
  QLineEdit* steer_input_e;

  QLabel* direction_input_l;
  QLineEdit* direction_input_e;

  QLabel* chara_speed_input_l;
  QLineEdit* chara_speed_input_e;

  QLabel* radar_status_l;
  QLineEdit* radar_status_e;

  QLabel* voltage_l;
  QLineEdit* voltage_e;

  QLabel* tempera_l;
  QLineEdit* tempera_e;

  QLabel* block_l;
  QLineEdit* block_e;

  QFont font;

  rclcpp::Node::SharedPtr node_{nullptr};

  SensorStatusMsg sensor_status_{};

  rclcpp::Subscription<PacketMsg>::SharedPtr sub_{nullptr};
};
}  // namespace radar_rviz_panel
