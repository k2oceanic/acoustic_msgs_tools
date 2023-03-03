#ifndef WATER_COLUMN_VIEW_H
#define WATER_COLUMN_VIEW_H

#include <QWidget>
#include "qcustomplot.h"
#include <rclcpp/node.hpp>
#include <acoustic_msgs/msg/raw_sonar_image.hpp>
#include <acoustic_msgs/msg/sonar_detections.hpp>
#include <qtimer.h>
#include "libInterpolate/Interpolate.hpp"
#include "rclcpp/executors.hpp"
//#include "ros/master.h"

namespace Ui {
class WaterColumnView;
}

class WaterColumnView : public QWidget, public rclcpp::Node
{
  Q_OBJECT

public:
  explicit WaterColumnView(QWidget *parent = nullptr);
  ~WaterColumnView();
  void setupSignals();

  void wcCallback(const acoustic_msgs::msg::RawSonarImage::SharedPtr wc_msg);
  void detectionCallback(const acoustic_msgs::msg::SonarDetections::SharedPtr det_msg);
private slots:
  void spinOnce();
  void updateRangeBearing(QMouseEvent *event);
  void setRange(double range);
  void updateTopics();

  void on_wc_topic_currentIndexChanged(const QString &arg1);

  void on_fullscreen_btn_clicked();

  void on_range_valueChanged(double arg1);

  void on_refresh_btn_clicked();

  void on_auto_gain_stateChanged(int state);

  void on_detect_topic_currentTextChanged(const QString &arg1);

  void on_color_ramp_select_currentIndexChanged(int index);

private:
  void checkFlipState();
  Ui::WaterColumnView *ui;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<acoustic_msgs::msg::RawSonarImage>::SharedPtr wc_sub_;
  rclcpp::Subscription<acoustic_msgs::msg::SonarDetections>::SharedPtr det_sub_;
  QTimer *ros_timer;
  QCPColorMap *colorMap;
  QCPGraph *detctionGraph;
  bool new_msg;
};

#endif // WATER_COLUMN_VIEW_H
