#ifndef WATER_COLUMN_VIEW_H
#define WATER_COLUMN_VIEW_H

#include <QWidget>
#include "qcustomplot.h"
#include <ros/ros.h>
#include <acoustic_msgs/RawSonarImage.h>
#include <qtimer.h>
#include "libInterpolate/Interpolate.hpp"
#include "ros/master.h"

namespace Ui {
class WaterColumnView;
}

class WaterColumnView : public QWidget
{
  Q_OBJECT

public:
  explicit WaterColumnView(QWidget *parent = nullptr);
  ~WaterColumnView();
  void setupSignals();

  void wcCallback(const acoustic_msgs::RawSonarImage::ConstPtr& wc_msg);
private slots:
  void spinOnce();
  void updateRangeBearing(QMouseEvent *event);
  void setRange(double range);
  void updateTopics();



  void on_wc_topic_currentIndexChanged(const QString &arg1);

  void on_fullscreen_btn_clicked();

  void on_range_valueChanged(double arg1);

  void on_refresh_btn_clicked();

private:
  Ui::WaterColumnView *ui;
  ros::NodeHandlePtr nh_;
  ros::Subscriber wc_sub_;
  QTimer *ros_timer;
  QCPColorMap *colorMap;
  bool new_msg;
};

#endif // WATER_COLUMN_VIEW_H
