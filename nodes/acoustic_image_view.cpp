#include <QApplication>
#include "water_column_view.h"

int main(int argc, char *argv[])
{

  ros::init(argc, argv, "acoustic_image_view",ros::init_options::AnonymousName);
  QApplication a(argc, argv);

  WaterColumnView w;
  w.setWindowTitle(QString::fromStdString(
                       ros::this_node::getName()));
  w.show();  
  return a.exec();
}
