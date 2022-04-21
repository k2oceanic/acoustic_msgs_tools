#include "water_column_view.h"
#include "ui_water_column_view.h"

WaterColumnView::WaterColumnView(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::WaterColumnView)
{
  ui->setupUi(this);
  nh_.reset(new ros::NodeHandle("~"));

  updateTopics();

  ros_timer = new QTimer(this);
  connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
  ros_timer->start(100);

  ui->plot->setInteractions(QCP::iRangeDrag|QCP::iRangeZoom); // this will also allow rescaling the color scale by dragging/zooming
  ui->plot->axisRect()->setupFullAxesBox(true);
  colorMap = new QCPColorMap(ui->plot->xAxis, ui->plot->yAxis);
  setRange(ui->range->value());

  setupSignals();
}

WaterColumnView::~WaterColumnView()
{
  delete ui;
}

void WaterColumnView::setupSignals(){
    QObject::connect(ui->plot, SIGNAL(mouseMove(QMouseEvent*)),
                     this,SLOT(updateRangeBearing(QMouseEvent*)));
}

double getRange(const acoustic_msgs::RawSonarImage::ConstPtr &wc_msg, size_t sample_number){
    double range = double(sample_number) *
            wc_msg->ping_info.sound_speed /
            (2.0 * wc_msg->sample_rate);
    return  range;
}

int getSampleNo(const acoustic_msgs::RawSonarImage::ConstPtr &wc_msg, double range){
    double scale = (2.0 * wc_msg->sample_rate) /
                       wc_msg->ping_info.sound_speed;
    int sample_no = range * scale;
    return sample_no;
}

double rowMajor(const acoustic_msgs::RawSonarImage::ConstPtr &wc_msg, double  beam_idx, double sample_idx){

  beam_idx = beam_idx + 0.5 - (beam_idx<0);
  sample_idx = sample_idx + 0.5 - (sample_idx<0);
  sample_idx = sample_idx - wc_msg->sample0;
  auto rows = wc_msg->samples_per_beam;
  auto cols = wc_msg->rx_angles.size();
  auto index = int(sample_idx)*int(cols)+int(beam_idx);

  if (int(beam_idx)>=cols || int(sample_idx)>=rows || int(beam_idx)<0 || int(sample_idx)<0 || index >= int(rows * cols) || index < 0){
    return 0.0;
  }else {
    switch( wc_msg->image.dtype){
      case acoustic_msgs::SonarImageData::DTYPE_UINT8:
        return reinterpret_cast<const uint8_t*>(wc_msg->image.data.data())[index];
      case acoustic_msgs::SonarImageData::DTYPE_INT8:
        return reinterpret_cast<const int8_t*>(wc_msg->image.data.data())[index];
      case acoustic_msgs::SonarImageData::DTYPE_UINT16:
        return reinterpret_cast<const uint16_t*>(wc_msg->image.data.data())[index];
      case acoustic_msgs::SonarImageData::DTYPE_INT16:
        return reinterpret_cast<const int16_t*>(wc_msg->image.data.data())[index];
      case acoustic_msgs::SonarImageData::DTYPE_UINT32:
        return reinterpret_cast<const uint32_t*>(wc_msg->image.data.data())[index];
      case acoustic_msgs::SonarImageData::DTYPE_INT32:
        return reinterpret_cast<const int32_t*>(wc_msg->image.data.data())[index];
      case acoustic_msgs::SonarImageData::DTYPE_UINT64:
        return reinterpret_cast<const uint64_t*>(wc_msg->image.data.data())[index];
      case acoustic_msgs::SonarImageData::DTYPE_INT64:
        return reinterpret_cast<const int64_t*>(wc_msg->image.data.data())[index];
      case acoustic_msgs::SonarImageData::DTYPE_FLOAT32:
        return reinterpret_cast<const float*>(wc_msg->image.data.data())[index];
      case acoustic_msgs::SonarImageData::DTYPE_FLOAT64:
        return reinterpret_cast<const double*>(wc_msg->image.data.data())[index];
      default:
        return std::nan(""); // unknown data type    
    }
  }
}

double getVal(const acoustic_msgs::RawSonarImage::ConstPtr &wc_msg,_1D::LinearInterpolator<double> beam_idx_interp, double x, double y){


  auto angle = atan2(x,y);
  if(wc_msg->rx_angles.size() == 1)
  {
    if(!wc_msg->ping_info.rx_beamwidths.empty() && fabs(angle) > wc_msg->ping_info.rx_beamwidths[0])
      return 0;
    else
    {
      x = getSampleNo(wc_msg,x);
      y = getSampleNo(wc_msg,y);
      auto v = std::sqrt(std::pow(x,2)+std::pow(y,2));
      return rowMajor(wc_msg,0,v);
    }

  }
  if(angle>wc_msg->rx_angles.back() || angle<wc_msg->rx_angles.front()){
    return 0;
  }else{

      x = getSampleNo(wc_msg,x);
      y = getSampleNo(wc_msg,y);


      auto u = beam_idx_interp(angle);
      auto v = std::sqrt(std::pow(x,2)+std::pow(y,2));
      auto out = rowMajor(wc_msg,u,v);

      return out;
  }
}




void WaterColumnView::wcCallback(const acoustic_msgs::RawSonarImage::ConstPtr &wc_msg){
  auto M = wc_msg->samples_per_beam;
  auto N = wc_msg->rx_angles.size();



  const uint8_t *bits = wc_msg->image.data.data();

  auto beam_angles = wc_msg->rx_angles;
  std::vector<float> beam_index;
  beam_index.resize(N);
  for(size_t i = 0; i< beam_index.size(); i++){
    beam_index[i] = i;
  }
  _1D::LinearInterpolator<double> beam_idx_interp;
  beam_idx_interp.setData(beam_angles,beam_index);

  if(new_msg){
    setRange(getRange(wc_msg,M+wc_msg->sample0));
    new_msg = false;
  }

  //ROS_INFO("wc callback");
  ui->plot->clearItems();

  ui->plot->yAxis->setRangeReversed(ui->reverse_y->checkState());
  ui->plot->xAxis->setRangeReversed(ui->reverse_x->checkState());


  int nx = 300;
  int ny = 400;
  colorMap->data()->setSize(nx, ny); // we want the color map to have nx * ny data points

  //double max_range = ui->range->value();

  ui->plot->yAxis->setScaleRatio(ui->plot->xAxis,1.0);
  colorMap->data()->setRange(QCPRange(ui->plot->xAxis->range().lower, ui->plot->xAxis->range().upper), QCPRange(ui->plot->yAxis->range().lower, ui->plot->yAxis->range().upper));

  double x, y, z;
  for (int xIndex=0; xIndex<nx; ++xIndex)
  {
    for (int yIndex=0; yIndex<ny; ++yIndex)
    {
      colorMap->data()->cellToCoord(xIndex, yIndex, &x, &y);
      z = getVal(wc_msg,beam_idx_interp,x,y);
      colorMap->data()->setCell(xIndex, yIndex, z);
    }
  }



  // set the color gradient of the color map to one of the presets:
  colorMap->setGradient(QCPColorGradient::gpHot);

  if(ui->auto_gain->isChecked())
    colorMap->rescaleDataRange(true);
  else
    colorMap->setDataRange(QCPRange(0,ui->gain->maximum() - ui->gain->value()));

  // make sure the axis rect and color scale synchronize their bottom and top margins (so they line up):
  QCPMarginGroup *marginGroup = new QCPMarginGroup(ui->plot);
  ui->plot->axisRect()->setMarginGroup(QCP::msBottom|QCP::msTop, marginGroup);

  ui->plot->replot();

  return;
}


void WaterColumnView::spinOnce(){
  if(ros::ok()){
    ros::spinOnce();
  }
  else
      QApplication::quit();
}


void WaterColumnView::on_wc_topic_currentIndexChanged(const QString &arg1)
{
  if(wc_sub_.getTopic() != arg1.toStdString()){
    wc_sub_ = nh_->subscribe<acoustic_msgs::RawSonarImage>(arg1.toStdString(), 1, &WaterColumnView::wcCallback, this);
    new_msg = true;
  }
}

void WaterColumnView::updateRangeBearing(QMouseEvent *event){
    QPoint p = event->pos();
    double x = ui->plot->xAxis->pixelToCoord(p.x());
    double y = ui->plot->yAxis->pixelToCoord(p.y());
    double range = std::sqrt(std::pow(x,2)+std::pow(y,2));
    double bearing = std::atan2(x,y)*180/3.14519;
    QString text;
    text.sprintf("Cursor Location:  x=%04.1f, y=%04.1f, range=%04.1f, bearing=%04.1f", x,y,range,bearing);
    ui->range_bearing->setText(text);
}

void WaterColumnView::on_fullscreen_btn_clicked()
{
    isFullScreen() ? showNormal() : showFullScreen();
}

void WaterColumnView::setRange(double range){
  ui->plot->yAxis->setRange(0,range);
  ui->plot->xAxis->setScaleRatio(ui->plot->yAxis,1.0);
  auto size = ui->plot->xAxis->range().size();
  ui->plot->xAxis->setRange(-size/2,size/2);
  ui->range->setValue(range);
}

void WaterColumnView::on_range_valueChanged(double arg1)
{
  setRange(arg1);
}

void WaterColumnView::updateTopics(){
  ros::master::V_TopicInfo master_topics;
  ros::master::getTopics(master_topics);
  ui->wc_topic->clear();
  QStringList topic_list;
  for(auto topic : master_topics){
    if(topic.datatype=="acoustic_msgs/RawSonarImage"){
      QString::fromStdString(topic.name);
      topic_list.push_back(QString::fromStdString(topic.name));
    }
  }
  ui->wc_topic->addItems(topic_list);
}

void WaterColumnView::on_refresh_btn_clicked()
{
  updateTopics();
}

void WaterColumnView::on_auto_gain_stateChanged(int state)
{
  if(state)
  {  
    ui->gain->setDisabled(true);
    colorMap->rescaleDataRange(true);
  }
  else
  {
    ui->gain->setEnabled(true);
    colorMap->setDataRange(QCPRange(0,ui->gain->maximum() - ui->gain->value()));
  }
  ui->plot->replot();
}