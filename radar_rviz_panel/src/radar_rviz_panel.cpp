/*
 * Copyright (c) 2020 Intel Corporation
 *
 * This work is licensed under the terms of the MIT license.
 * For a copy, see <https://opensource.org/licenses/MIT>.
 */
#include "radar_rviz_panel/radar_rviz_panel.h"

#include <OgreCamera.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>
#include <math.h>
#include <tf2/utils.h>

#include <QCheckBox>
#include <QComboBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QPixmap>
#include <QProgressBar>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <QScrollArea>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <iomanip>
#include <rviz_common/view_manager.hpp>

#include "rviz_common/display_context.hpp"

namespace radar_rviz_panel {

RadarRvizPanel::RadarRvizPanel(QWidget *parent) : rviz_common::Panel(parent) {
  memset(&sensor_status_, 0, sizeof(SensorStatusMsg));

  font = QFont("Microsoft YaHei", 10, QFont::Bold);

  // 布局
  QGridLayout *g_layout = new QGridLayout;
  QScrollArea *scrollArea = new QScrollArea(this);
  QWidget *contentWidget = new QWidget;
  QVBoxLayout *layout = new QVBoxLayout;

  int rows = 0;

  sec_l = new QLabel("秒: ");
  sec_e = new QLineEdit;
  // SetLabeltStyle(sec_l, font, 200, Qt::AlignCenter, "QLabel {color : #000000; }");
  sec_e->setReadOnly(true);
  g_layout->addWidget(sec_l, rows, 0);
  g_layout->addWidget(sec_e, rows, 1);

  rows++;
  nano_l = new QLabel("纳秒: ");
  nano_e = new QLineEdit;
  // SetLabeltStyle(nano_l, font, 200, Qt::AlignCenter, "QLabel {color : #000000; }");
  nano_e->setReadOnly(true);
  g_layout->addWidget(nano_l, rows, 0);
  g_layout->addWidget(nano_e, rows, 1);

  rows++;
  sync_l = new QLabel("传感器状态消息时间戳同步状态: ");
  sync_e = new QLineEdit;
  // SetLabeltStyle(sync_l, font, 200, Qt::AlignCenter, "QLabel {color : #000000; }");
  sync_e->setReadOnly(true);
  g_layout->addWidget(sync_l, rows, 0);
  g_layout->addWidget(sync_e, rows, 1);

  rows++;
  version_l = new QLabel("软件版本: ");
  version_e = new QLineEdit;
  // SetLabeltStyle(version_l, font, 200, Qt::AlignCenter, "QLabel {color : #000000; }");
  version_e->setReadOnly(true);
  g_layout->addWidget(version_l, rows, 0);
  g_layout->addWidget(version_e, rows, 1);

  rows++;
  lon_position_l = new QLabel("雷达纵向位置 (单位:m): ");
  lon_position_e = new QLineEdit;
  // SetLabeltStyle(lon_position_l, font, 200, Qt::AlignCenter, "QLabel {color : #000000; }");
  lon_position_e->setReadOnly(true);
  g_layout->addWidget(lon_position_l, rows, 0);
  g_layout->addWidget(lon_position_e, rows, 1);

  rows++;
  lat_position_l = new QLabel("雷达横向位置 (单位:m): ");
  lat_position_e = new QLineEdit;
  // SetLabeltStyle(lat_position_l, font, 200, Qt::AlignCenter, "QLabel {color : #000000; }");
  lat_position_e->setReadOnly(true);
  g_layout->addWidget(lat_position_l, rows, 0);
  g_layout->addWidget(lat_position_e, rows, 1);

  rows++;
  ver_position_l = new QLabel("雷达高度 (单位:m): ");
  ver_position_e = new QLineEdit;
  // SetLabeltStyle(ver_position_l, font, 200, Qt::AlignCenter, "QLabel {color : #000000; }");
  ver_position_e->setReadOnly(true);
  g_layout->addWidget(ver_position_l, rows, 0);
  g_layout->addWidget(ver_position_e, rows, 1);

  rows++;
  yaw_l = new QLabel("雷达yaw角 (单位:rad): ");
  yaw_e = new QLineEdit;
  // SetLabeltStyle(yaw_l, font, 200, Qt::AlignCenter, "QLabel {color : #000000; }");
  yaw_e->setReadOnly(true);
  g_layout->addWidget(yaw_l, rows, 0);
  g_layout->addWidget(yaw_e, rows, 1);

  rows++;
  pitch_l = new QLabel("雷达pitch (单位:rad): ");
  pitch_e = new QLineEdit;
  // SetLabeltStyle(pitch_l, font, 200, Qt::AlignCenter, "QLabel {color : #000000; }");
  pitch_e->setReadOnly(true);
  g_layout->addWidget(pitch_l, rows, 0);
  g_layout->addWidget(pitch_e, rows, 1);

  rows++;
  orientation_plug_l = new QLabel("雷达连接器朝向: ");
  orientation_plug_e = new QLineEdit;
  // SetLabeltStyle(orientation_plug_l, font, 200, Qt::AlignCenter, "QLabel {color : #000000; }");
  orientation_plug_e->setReadOnly(true);
  g_layout->addWidget(orientation_plug_l, rows, 0);
  g_layout->addWidget(orientation_plug_e, rows, 1);

  rows++;
  vehicle_length_l = new QLabel("本车长度 (单位:m): ");
  vehicle_length_e = new QLineEdit;
  // SetLabeltStyle(vehicle_length_l, font, 200, Qt::AlignCenter, "QLabel {color : #000000; }");
  vehicle_length_e->setReadOnly(true);
  g_layout->addWidget(vehicle_length_l, rows, 0);
  g_layout->addWidget(vehicle_length_e, rows, 1);

  rows++;
  vehicle_width_l = new QLabel("本车宽度 (单位:m): ");
  vehicle_width_e = new QLineEdit;
  // SetLabeltStyle(vehicle_width_l, font, 200, Qt::AlignCenter, "QLabel {color : #000000; }");
  vehicle_width_e->setReadOnly(true);
  g_layout->addWidget(vehicle_width_l, rows, 0);
  g_layout->addWidget(vehicle_width_e, rows, 1);

  rows++;
  vehicle_height_l = new QLabel("本车高度 (单位:m): ");
  vehicle_height_e = new QLineEdit;
  // SetLabeltStyle(vehicle_height_l, font, 200, Qt::AlignCenter, "QLabel {color : #000000; }");
  vehicle_height_e->setReadOnly(true);
  g_layout->addWidget(vehicle_height_l, rows, 0);
  g_layout->addWidget(vehicle_height_e, rows, 1);

  rows++;
  vehicle_wheelbase_l = new QLabel("本车轴距 (单位:m): ");
  vehicle_wheelbase_e = new QLineEdit;
  // SetLabeltStyle(vehicle_wheelbase_l, font, 200, Qt::AlignCenter, "QLabel {color : #000000; }");
  vehicle_wheelbase_e->setReadOnly(true);
  g_layout->addWidget(vehicle_wheelbase_l, rows, 0);
  g_layout->addWidget(vehicle_wheelbase_e, rows, 1);

  rows++;
  max_detec_distance_l = new QLabel("最大探测距离 (单位:m): ");
  max_detec_distance_e = new QLineEdit;
  // SetLabeltStyle(max_detec_distance_l, font, 200, Qt::AlignCenter, "QLabel {color : #000000; }");
  max_detec_distance_e->setReadOnly(true);
  g_layout->addWidget(max_detec_distance_l, rows, 0);
  g_layout->addWidget(max_detec_distance_e, rows, 1);

  rows++;
  center_frequency_l = new QLabel("中心频率: ");
  center_frequency_e = new QLineEdit;
  // SetLabeltStyle(center_frequency_l, font, 200, Qt::AlignCenter, "QLabel {color : #000000; }");
  center_frequency_e->setReadOnly(true);
  g_layout->addWidget(center_frequency_l, rows, 0);
  g_layout->addWidget(center_frequency_e, rows, 1);

  rows++;
  cycle_time_l = new QLabel("测量周期(雷达扫描周期,单位:ms): ");
  cycle_time_e = new QLineEdit;
  // SetLabeltStyle(cycle_time_l, font, 200, Qt::AlignCenter, "QLabel {color : #000000; }");
  cycle_time_e->setReadOnly(true);
  g_layout->addWidget(cycle_time_l, rows, 0);
  g_layout->addWidget(cycle_time_e, rows, 1);

  rows++;
  cycle_offest_l = new QLabel("测量周期偏移量（单位:ms): ");
  cycle_offest_e = new QLineEdit;
  // SetLabeltStyle(cycle_offest_l, font, 200, Qt::AlignCenter, "QLabel {color : #000000; }");
  cycle_offest_e->setReadOnly(true);
  g_layout->addWidget(cycle_offest_l, rows, 0);
  g_layout->addWidget(cycle_offest_e, rows, 1);

  rows++;
  country_code_l = new QLabel("国家码: ");
  country_code_e = new QLineEdit;
  // SetLabeltStyle(country_code_l, font, 200, Qt::AlignCenter, "QLabel {color : #000000; }");
  country_code_e->setReadOnly(true);
  g_layout->addWidget(country_code_l, rows, 0);
  g_layout->addWidget(country_code_e, rows, 1);

  rows++;
  power_mode_l = new QLabel("车辆静止时雷达进入低功耗: ");
  power_mode_e = new QLineEdit;
  // SetLabeltStyle(power_mode_l, font, 200, Qt::AlignCenter, "QLabel {color : #000000; }");
  power_mode_e->setReadOnly(true);
  g_layout->addWidget(power_mode_l, rows, 0);
  g_layout->addWidget(power_mode_e, rows, 1);

  rows++;
  ip_l = new QLabel("雷达IP地址: ");
  ip_e = new QLineEdit;
  // SetLabeltStyle(ip_l, font, 200, Qt::AlignCenter, "QLabel {color : #000000; }");
  ip_e->setReadOnly(true);
  g_layout->addWidget(ip_l, rows, 0);
  g_layout->addWidget(ip_e, rows, 1);

  rows++;
  ip_reserve_l = new QLabel("保留IP地址: ");
  ip_reserve_e = new QLineEdit;
  // SetLabeltStyle(ip_reserve_l, font, 200, Qt::AlignCenter, "QLabel {color : #000000; }");
  ip_reserve_e->setReadOnly(true);
  g_layout->addWidget(ip_reserve_l, rows, 0);
  g_layout->addWidget(ip_reserve_e, rows, 1);

  rows++;
  cfg_counter_l = new QLabel("本次上电后合法配置次数: ");
  cfg_counter_e = new QLineEdit;
  // SetLabeltStyle(cfg_counter_l, font, 200, Qt::AlignCenter, "QLabel {color : #000000; }");
  cfg_counter_e->setReadOnly(true);
  g_layout->addWidget(cfg_counter_l, rows, 0);
  g_layout->addWidget(cfg_counter_e, rows, 1);

  rows++;
  speed_input_l = new QLabel("车速信息输入状态: ");
  speed_input_e = new QLineEdit;
  // SetLabeltStyle(speed_input_l, font, 200, Qt::AlignCenter, "QLabel {color : #000000; }");
  speed_input_e->setReadOnly(true);
  g_layout->addWidget(speed_input_l, rows, 0);
  g_layout->addWidget(speed_input_e, rows, 1);

  rows++;
  lon_acc_input_l = new QLabel("纵向加速度信息输入状态: ");
  lon_acc_input_e = new QLineEdit;
  // SetLabeltStyle(lon_acc_input_l, font, 200, Qt::AlignCenter, "QLabel {color : #000000; }");
  lon_acc_input_e->setReadOnly(true);
  g_layout->addWidget(lon_acc_input_l, rows, 0);
  g_layout->addWidget(lon_acc_input_e, rows, 1);

  rows++;
  lat_acc_input_l = new QLabel("横向加速度信息输入状态: ");
  lat_acc_input_e = new QLineEdit;
  // SetLabeltStyle(lat_acc_input_l, font, 200, Qt::AlignCenter, "QLabel {color : #000000; }");
  lat_acc_input_e->setReadOnly(true);
  g_layout->addWidget(lat_acc_input_l, rows, 0);
  g_layout->addWidget(lat_acc_input_e, rows, 1);

  rows++;
  yawrate_input_l = new QLabel("偏航率信息输入状态: ");
  yawrate_input_e = new QLineEdit;
  // SetLabeltStyle(yawrate_input_l, font, 200, Qt::AlignCenter, "QLabel {color : #000000; }");
  yawrate_input_e->setReadOnly(true);
  g_layout->addWidget(yawrate_input_l, rows, 0);
  g_layout->addWidget(yawrate_input_e, rows, 1);

  rows++;
  steer_input_l = new QLabel("方向盘转角信息输入状态: ");
  steer_input_e = new QLineEdit;
  // SetLabeltStyle(steer_input_l, font, 200, Qt::AlignCenter, "QLabel {color : #000000; }");
  steer_input_e->setReadOnly(true);
  g_layout->addWidget(steer_input_l, rows, 0);
  g_layout->addWidget(steer_input_e, rows, 1);

  rows++;
  direction_input_l = new QLabel("车辆行驶方向信息输入状态: ");
  direction_input_e = new QLineEdit;
  // SetLabeltStyle(direction_input_l, font, 200, Qt::AlignCenter, "QLabel {color : #000000; }");
  direction_input_e->setReadOnly(true);
  g_layout->addWidget(direction_input_l, rows, 0);
  g_layout->addWidget(direction_input_e, rows, 1);

  rows++;
  chara_speed_input_l = new QLabel("当前车辆特征速度信息输入状态: ");
  chara_speed_input_e = new QLineEdit;
  // SetLabeltStyle(chara_speed_input_l, font, 200, Qt::AlignCenter, "QLabel {color : #000000; }");
  chara_speed_input_e->setReadOnly(true);
  g_layout->addWidget(chara_speed_input_l, rows, 0);
  g_layout->addWidget(chara_speed_input_e, rows, 1);

  rows++;
  radar_status_l = new QLabel("雷达状态: ");
  radar_status_e = new QLineEdit;
  // SetLabeltStyle(radar_status_l, font, 200, Qt::AlignCenter, "QLabel {color : #000000; }");
  radar_status_e->setReadOnly(true);
  g_layout->addWidget(radar_status_l, rows, 0);
  g_layout->addWidget(radar_status_e, rows, 1);

  rows++;
  voltage_l = new QLabel("电压状态: ");
  voltage_e = new QLineEdit;
  // SetLabeltStyle(voltage_l, font, 200, Qt::AlignCenter, "QLabel {color : #000000; }");
  voltage_e->setReadOnly(true);
  g_layout->addWidget(voltage_l, rows, 0);
  g_layout->addWidget(voltage_e, rows, 1);

  rows++;
  tempera_l = new QLabel("温度状态: ");
  tempera_e = new QLineEdit;
  // SetLabeltStyle(tempera_l, font, 200, Qt::AlignCenter, "QLabel {color : #000000; }");
  tempera_e->setReadOnly(true);
  g_layout->addWidget(tempera_l, rows, 0);
  g_layout->addWidget(tempera_e, rows, 1);

  rows++;
  block_l = new QLabel("阻塞和阻塞自检状态: ");
  block_e = new QLineEdit;
  // SetLabeltStyle(block_l, font, 200, Qt::AlignCenter, "QLabel {color : #000000; }");
  block_e->setReadOnly(true);
  g_layout->addWidget(block_l, rows, 0);
  g_layout->addWidget(block_e, rows, 1);

  contentWidget->setLayout(g_layout);
  scrollArea->setWidget(contentWidget);
  scrollArea->setWidgetResizable(true);

  layout->addWidget(scrollArea);
  setLayout(layout);
}

void RadarRvizPanel::onInitialize() {
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  // set up ros subscriber and publishers
  sub_ = node_->create_subscription<PacketMsg>(
      "/sensor/image_radar0_status", 10,
      std::bind(&RadarRvizPanel::SensorStatusChanged, this, std::placeholders::_1));
}

void RadarRvizPanel::SetLabeltStyle(QLabel *label, QFont font, int width, Qt::Alignment aalignment,
                                    QString color) {
  label->setFont(font);
  label->setFixedWidth(width);
  label->setAlignment(aalignment);
  label->setStyleSheet(color);
}

void RadarRvizPanel::SensorStatusChanged(const PacketMsg::SharedPtr msg) {
  if (msg->type != 2) {
    return;
  }

  if (msg->size != 84) {
    std::cerr << "Recv a sensor status msg, but its size is not 84" << std::endl;
    return;
  }

  memcpy(&sensor_status_, msg->data.data(), 84);

  QString str = QString("%1").arg(sensor_status_.sensor_status.timestamp_s);
  sec_e->setText(str);

  str = QString("%1").arg(sensor_status_.sensor_status.timestamp_n);
  nano_e->setText(str);

  if (sensor_status_.sensor_status.timestamp_sync_status == 1) {
    str = QString("%1").arg("同步正常");
  } else if (sensor_status_.sensor_status.timestamp_sync_status == 2) {
    str = QString("%1").arg("未同步");
  } else {
    str = QString("%1").arg("同步终止");
  }
  sync_e->setText(str);

  str = QString("%1.%2.%3")
            .arg(QString::number(sensor_status_.sensor_status.version_major, 16))
            .arg(QString::number(sensor_status_.sensor_status.version_minor, 16))
            .arg(QString::number(sensor_status_.sensor_status.version_patch, 16));
  version_e->setText(str);

  str = QString("%1").arg(sensor_status_.sensor_status.longitudinal);
  lon_position_e->setText(str);

  str = QString("%1").arg(sensor_status_.sensor_status.lateral);
  lat_position_e->setText(str);

  str = QString("%1").arg(sensor_status_.sensor_status.vertical);
  ver_position_e->setText(str);

  str = QString("%1").arg(sensor_status_.sensor_status.yaw);
  yaw_e->setText(str);

  str = QString("%1").arg(sensor_status_.sensor_status.pitch);
  pitch_e->setText(str);

  if (sensor_status_.sensor_status.plug_orientation == 0) {
    str = QString("%1").arg("朝向驾驶员左侧");
  } else {
    str = QString("%1").arg("朝向驾驶员右侧");
  }
  orientation_plug_e->setText(str);

  str = QString("%1").arg(sensor_status_.sensor_status.length);
  vehicle_length_e->setText(str);
  str = QString("%1").arg(sensor_status_.sensor_status.width);
  vehicle_width_e->setText(str);
  str = QString("%1").arg(sensor_status_.sensor_status.height);
  vehicle_height_e->setText(str);
  str = QString("%1").arg(sensor_status_.sensor_status.wheelbase);
  vehicle_wheelbase_e->setText(str);
  str = QString("%1").arg(sensor_status_.sensor_status.max_distance);
  max_detec_distance_e->setText(str);

  if (sensor_status_.sensor_status.frequency_slot == 0) {
    str = QString("%1").arg("Low(76.23GHz)");
  } else if (sensor_status_.sensor_status.frequency_slot == 1) {
    str = QString("%1").arg("Mid(76.48GHz)");
  } else if (sensor_status_.sensor_status.frequency_slot == 1) {
    str = QString("%1").arg("High(76.73GHz)");
  } else {
    str = QString("%1").arg("");
  }
  center_frequency_e->setText(str);

  str = QString("%1").arg(sensor_status_.sensor_status.cycle_time);
  cycle_time_e->setText(str);

  str = QString("%1").arg(sensor_status_.sensor_status.time_slot);
  cycle_offest_e->setText(str);

  if (sensor_status_.sensor_status.hcc == 1) {
    str = QString("%1").arg("WorldWide(世界通用版)");
  } else {
    str = QString("%1").arg("Japan(日本市场版本)");
  }
  country_code_e->setText(str);

  if (sensor_status_.sensor_status.power_save_standstill == 0) {
    str = QString("%1").arg("关闭");
  } else {
    str = QString("%1").arg("开启");
  }
  power_mode_e->setText(str);

  str = QString("%1.%2.%3.%4")
            .arg(QString::number((sensor_status_.sensor_status.sensor_ip_0 & 0xFF000000) >> 24u))
            .arg(QString::number((sensor_status_.sensor_status.sensor_ip_0 & 0x00FF0000) >> 16u))
            .arg(QString::number((sensor_status_.sensor_status.sensor_ip_0 & 0x0000FF00) >> 8u))
            .arg(QString::number(sensor_status_.sensor_status.sensor_ip_0 & 0x000000FF));
  ip_e->setText(str);

  str = QString("%1.%2.%3.%4")
            .arg(QString::number((sensor_status_.sensor_status.sensor_ip_1 & 0xFF000000) >> 24u))
            .arg(QString::number((sensor_status_.sensor_status.sensor_ip_1 & 0x00FF0000) >> 16u))
            .arg(QString::number((sensor_status_.sensor_status.sensor_ip_1 & 0x0000FF00) >> 8u))
            .arg(QString::number(sensor_status_.sensor_status.sensor_ip_1 & 0x000000FF));
  ip_reserve_e->setText(str);

  str = QString("%1").arg(QString::number(sensor_status_.sensor_status.configuration_counter));
  cfg_counter_e->setText(str);

  if (sensor_status_.sensor_status.status_longitudinal_velocity == 0) {
    str = QString("%1").arg("输入正常");
  } else {
    str = QString("%1").arg("输入超时");
  }
  speed_input_e->setText(str);

  if (sensor_status_.sensor_status.status_longitudinal_acceleration == 0) {
    str = QString("%1").arg("输入正常");
  } else {
    str = QString("%1").arg("输入超时");
  }
  lon_acc_input_e->setText(str);

  if (sensor_status_.sensor_status.status_lateral_acceleration == 0) {
    str = QString("%1").arg("输入正常");
  } else {
    str = QString("%1").arg("输入超时");
  }
  lat_acc_input_e->setText(str);

  if (sensor_status_.sensor_status.status_yaw_rate == 0) {
    str = QString("%1").arg("输入正常");
  } else {
    str = QString("%1").arg("输入超时");
  }
  yawrate_input_e->setText(str);

  if (sensor_status_.sensor_status.status_steering_angle == 0) {
    str = QString("%1").arg("输入正常");
  } else {
    str = QString("%1").arg("输入超时");
  }
  steer_input_e->setText(str);

  if (sensor_status_.sensor_status.status_driving_direction == 0) {
    str = QString("%1").arg("输入正常");
  } else {
    str = QString("%1").arg("输入超时");
  }
  direction_input_e->setText(str);

  if (sensor_status_.sensor_status.status_characteristic_speed == 0) {
    str = QString("%1").arg("输入正常");
  } else {
    str = QString("%1").arg("(暂未使用)输入超时");
  }
  chara_speed_input_e->setText(str);

  if (sensor_status_.sensor_status.status_radar_status == 0) {
    str = QString("%1").arg("雷达正在初始化");
  } else if (sensor_status_.sensor_status.status_radar_status == 1) {
    str = QString("%1").arg("雷达状态正常");
  } else if (sensor_status_.sensor_status.status_radar_status == 2) {
    str = QString("%1").arg("雷达状态异常");
  } else {
    str = QString("%1").arg(QString::number(sensor_status_.sensor_status.status_radar_status));
  }
  radar_status_e->setText(str);

  if (sensor_status_.sensor_status.status_voltage_status == 0x0) {
    str = QString("%1").arg("电压正常");
  } else if (sensor_status_.sensor_status.status_voltage_status == 0x1) {
    str = QString("%1").arg("当前欠压");
  } else if (sensor_status_.sensor_status.status_voltage_status == 0x2) {
    str = QString("%1").arg("之前欠压");
  } else if (sensor_status_.sensor_status.status_voltage_status == 0x4) {
    str = QString("%1").arg("当前过压");
  } else if (sensor_status_.sensor_status.status_voltage_status == 0x8) {
    str = QString("%1").arg("之前过压");
  } else {
    str =
        QString("%1").arg(QString::number(sensor_status_.sensor_status.status_voltage_status, 16));
  }
  voltage_e->setText(str);

  if (sensor_status_.sensor_status.status_temperature_status == 0x0) {
    str = QString("%1").arg("温度正常");
  } else if (sensor_status_.sensor_status.status_temperature_status == 0x1) {
    str = QString("%1").arg("当前低温");
  } else if (sensor_status_.sensor_status.status_temperature_status == 0x2) {
    str = QString("%1").arg("之前低温");
  } else if (sensor_status_.sensor_status.status_temperature_status == 0x4) {
    str = QString("%1").arg("当前高温");
  } else if (sensor_status_.sensor_status.status_temperature_status == 0x8) {
    str = QString("%1").arg("之前高温");
  } else {
    str = QString("%1").arg(
        QString::number(sensor_status_.sensor_status.status_temperature_status, 16));
  }
  tempera_e->setText(str);

  auto block_first = sensor_status_.sensor_status.status_blockage_status & 0xF0;
  auto block_second = sensor_status_.sensor_status.status_blockage_status & 0x0F;

  str = QString("");
  if (block_first == 0x00) {
    str += "堵塞自检失败";
  } else if (block_first == 0x10) {
    str += "堵塞自检通过";
  } else if (block_first == 0x20) {
    str += "堵塞自检进行中...";
  } else {
  }

  str += " ";
  if (block_second == 0x00) {
    str += "完全堵塞";
  } else if (block_second == 0x01) {
    str += "毒素程度高";
  } else if (block_second == 0x02) {
    str += "毒素程度中";
  } else if (block_second == 0x03) {
    str += "毒素程度低";
  } else if (block_second == 0x04) {
    str += "无堵塞";
  } else {
  }
  block_e->setText(str);
}

}  // namespace radar_rviz_panel
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(radar_rviz_panel::RadarRvizPanel, rviz_common::Panel)
