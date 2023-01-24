#include <QApplication>
#include "water_column_view.h"

int main(int argc, char *argv[])
{

//  ros::init(argc, argv, "acoustic_image_view",ros::init_options::AnonymousName);
  rclcpp::init(argc, argv);
  QApplication a(argc, argv);

  WaterColumnView w;
//  w.setWindowTitle(QString::fromStdString(
//                       ros::this_node::getName()));

  QIcon icon(":/icons/acoustic_image_view_icon.svg");
  w.setWindowIcon(icon);

  w.show();  
  return a.exec();
}
