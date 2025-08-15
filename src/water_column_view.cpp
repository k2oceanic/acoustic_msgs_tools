#include "water_column_view.h"
#include "ui_water_column_view.h"
#include <cmath>

namespace {
// Helpers for proper two-way travel time (TWTT) handling
inline double twtt_to_range(double twtt_sec, double c_mps) {
  // range [m] = 0.5 * c * (two-way time)
  return 0.5 * c_mps * twtt_sec;
}
inline double range_to_twtt(double range_m, double c_mps) {
  // two-way time [s] = 2 * range / c
  return (2.0 * range_m) / c_mps;
}
inline double sample_to_twtt(double sample_index, double sample_rate_hz) {
  // sample_index is an index counting from 0; time = samples / Fs
  return sample_index / sample_rate_hz;
}
inline double twtt_to_sample(double twtt_sec, double sample_rate_hz) {
  return twtt_sec * sample_rate_hz;
}
constexpr double kPI = 3.14159265358979323846;
}

WaterColumnView::WaterColumnView(QWidget *parent) :
    QWidget(parent),
    Node("water_column_view"),
    ui(new Ui::WaterColumnView)
{
  ui->setupUi(this);

  node_ = std::make_shared<rclcpp::Node>("hello_gui");

  updateTopics();

  ros_timer = new QTimer(this);
  connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
  ros_timer->start(10);

  ui->plot->setInteractions(QCP::iRangeDrag|QCP::iRangeZoom); // this will also allow rescaling the color scale by dragging/zooming
  ui->plot->axisRect()->setupFullAxesBox(true);
  colorMap = new QCPColorMap(ui->plot->xAxis, ui->plot->yAxis);
  colorMap->setGradient(QCPColorGradient::gpNight);
  detctionGraph = ui->plot->addGraph();

  setRange(ui->range->value());

  setupSignals();

  setWindowTitle(QString::fromStdString(this->get_name()));
}

WaterColumnView::~WaterColumnView()
{
  delete ui;
}

void WaterColumnView::setupSignals(){
  QObject::connect(ui->plot, SIGNAL(mouseMove(QMouseEvent*)),
                   this,SLOT(updateRangeBearing(QMouseEvent*)));
}

// Map sample index -> range using TWTT rigorously.
double getRange(const marine_acoustic_msgs::msg::RawSonarImage::SharedPtr wc_msg, size_t sample_number){
  const double twtt = sample_to_twtt(static_cast<double>(sample_number), wc_msg->sample_rate);
  return twtt_to_range(twtt, wc_msg->ping_info.sound_speed);
}

// Map range -> sample index using TWTT rigorously.
int getSampleNo(const marine_acoustic_msgs::msg::RawSonarImage::SharedPtr wc_msg, double range){
  const double twtt = range_to_twtt(range, wc_msg->ping_info.sound_speed);
  const double sample_no = twtt_to_sample(twtt, wc_msg->sample_rate);
  return static_cast<int>(std::lround(sample_no));
}

double rowMajor(const marine_acoustic_msgs::msg::RawSonarImage::SharedPtr wc_msg, double  beam_idx, double sample_idx){

  beam_idx = beam_idx + 0.5 - (beam_idx<0);
  sample_idx = sample_idx + 0.5 - (sample_idx<0);
  sample_idx = sample_idx - wc_msg->sample0; // stored image starts at sample0
  auto rows = wc_msg->samples_per_beam;
  auto cols = wc_msg->rx_angles.size();
  auto index = int(sample_idx)*int(cols)+int(beam_idx);

  if (int(beam_idx)>=int(cols) || int(sample_idx)>=int(rows) || int(beam_idx)<0 || int(sample_idx)<0 || index >= int(rows * cols) || index < 0){
    return 0.0;
  }else {
    switch( wc_msg->image.dtype){
    case marine_acoustic_msgs::msg::SonarImageData::DTYPE_UINT8:
      return reinterpret_cast<const uint8_t*>(wc_msg->image.data.data())[index];
    case marine_acoustic_msgs::msg::SonarImageData::DTYPE_INT8:
      return reinterpret_cast<const int8_t*>(wc_msg->image.data.data())[index];
    case marine_acoustic_msgs::msg::SonarImageData::DTYPE_UINT16:
      return reinterpret_cast<const uint16_t*>(wc_msg->image.data.data())[index];
    case marine_acoustic_msgs::msg::SonarImageData::DTYPE_INT16:
      return reinterpret_cast<const int16_t*>(wc_msg->image.data.data())[index];
    case marine_acoustic_msgs::msg::SonarImageData::DTYPE_UINT32:
      return reinterpret_cast<const uint32_t*>(wc_msg->image.data.data())[index];
    case marine_acoustic_msgs::msg::SonarImageData::DTYPE_INT32:
      return reinterpret_cast<const int32_t*>(wc_msg->image.data.data())[index];
    case marine_acoustic_msgs::msg::SonarImageData::DTYPE_UINT64:
      return reinterpret_cast<const uint64_t*>(wc_msg->image.data.data())[index];
    case marine_acoustic_msgs::msg::SonarImageData::DTYPE_INT64:
      return reinterpret_cast<const int64_t*>(wc_msg->image.data.data())[index];
    case marine_acoustic_msgs::msg::SonarImageData::DTYPE_FLOAT32:
      return reinterpret_cast<const float*>(wc_msg->image.data.data())[index];
    case marine_acoustic_msgs::msg::SonarImageData::DTYPE_FLOAT64:
      return reinterpret_cast<const double*>(wc_msg->image.data.data())[index];
    default:
      return std::nan(""); // unknown data type
    }
  }
}

double getVal(const marine_acoustic_msgs::msg::RawSonarImage::SharedPtr wc_msg,_1D::LinearInterpolator<double> beam_idx_interp, double x, double y){

  auto angle = atan2(x,y);
  if(wc_msg->rx_angles.size() == 1)
  {
    if(!wc_msg->ping_info.rx_beamwidths.empty() && fabs(angle) > wc_msg->ping_info.rx_beamwidths[0])
      return 0;
    else
    {
      // Convert Cartesian range to sample index via TWTT
      const double range_m = std::sqrt(x*x + y*y);
      const int sample_idx = getSampleNo(wc_msg, range_m);
      return rowMajor(wc_msg, 0, sample_idx);
    }
  }

  if(angle>wc_msg->rx_angles.back() || angle<wc_msg->rx_angles.front()){
    return 0;
  }else{
    // Convert Cartesian range to sample index via TWTT
    const double range_m = std::sqrt(x*x + y*y);
    const int v = getSampleNo(wc_msg, range_m);

    auto u = beam_idx_interp(angle); // fractional beam index
    auto out = rowMajor(wc_msg,u,v);
    return out;
  }
}

void WaterColumnView::checkFlipState(){
  ui->plot->yAxis->setRangeReversed(ui->reverse_y->checkState());
  ui->plot->xAxis->setRangeReversed(ui->reverse_x->checkState());
}

void WaterColumnView::wcCallback(const marine_acoustic_msgs::msg::RawSonarImage::SharedPtr wc_msg){
  auto M = wc_msg->samples_per_beam;
  auto N = wc_msg->rx_angles.size();

  auto beam_angles = wc_msg->rx_angles;
  std::vector<float> beam_index;
  beam_index.resize(N);
  for(size_t i = 0; i< beam_index.size(); i++){
    beam_index[i] = static_cast<float>(i);
  }
  _1D::LinearInterpolator<double> beam_idx_interp;
  beam_idx_interp.setData(beam_angles,beam_index);

  if(new_msg){
    // Set plot range to max range from last sample (accounting for sample0) using TWTT mapping
    setRange(getRange(wc_msg, static_cast<size_t>(M + wc_msg->sample0)));
    new_msg = false;
  }

  ui->plot->clearItems();
  checkFlipState();

  int nx = 300;
  int ny = 400;
  colorMap->data()->setSize(nx, ny); // we want the color map to have nx * ny data points

  ui->plot->yAxis->setScaleRatio(ui->plot->xAxis,1.0);
  colorMap->data()->setRange(QCPRange(ui->plot->xAxis->range().lower, ui->plot->xAxis->range().upper),
                             QCPRange(ui->plot->yAxis->range().lower, ui->plot->yAxis->range().upper));

  double x, y, z;
  for (int xIndex=0; xIndex<nx; ++xIndex)
  {
    for (int yIndex=0; yIndex<ny; ++yIndex)
    {
      colorMap->data()->cellToCoord(xIndex, yIndex, &x, &y); // x,y in meters
      z = getVal(wc_msg,beam_idx_interp,x,y);
      colorMap->data()->setCell(xIndex, yIndex, z);
    }
  }

  colorMap->setInterpolate(false);

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

// Proper TWTT usage: range = 0.5 * c * two_way_travel_time
void WaterColumnView::detectionCallback(const marine_acoustic_msgs::msg::SonarDetections::SharedPtr det_msg){
  auto n = det_msg->two_way_travel_times.size();
  auto sound_speed = det_msg->ping_info.sound_speed;
  checkFlipState();
  QVector<double> x(n), y(n);
  for(size_t i=0; i<n; i++){
    const double twtt = det_msg->two_way_travel_times[i]; // [s], round-trip
    const double range = twtt_to_range(twtt, sound_speed); // [m]
    const double rx_angle = det_msg->rx_angles[i];
    x[static_cast<int>(i)] = range * std::sin(rx_angle);
    y[static_cast<int>(i)] = range * std::cos(rx_angle);
  }
  detctionGraph->setData(x, y);
  QPen pen;
  pen.setColor(QColor(QColorConstants::Green));
  ui->plot->graph()->setPen(pen);
  ui->plot->graph()->setLineStyle((QCPGraph::LineStyle::lsNone));
  ui->plot->graph()->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssPlus, 4));

  ui->plot->replot();
}

void WaterColumnView::rangesCallback(const marine_acoustic_msgs::msg::SonarRanges::SharedPtr rng_msg){
  auto n = rng_msg->ranges.size();
  checkFlipState();
  QVector<double> x(n), y(n);
  for(size_t i=0; i<n; i++){
    double range = rng_msg->ranges[i]; // already meters (one-way range)
    double rx_angle = rng_msg->beam_unit_vec[i].x; // TODO: Validate it's changing along the x axis
    x[static_cast<int>(i)] = range * std::sin(rx_angle);
    y[static_cast<int>(i)] = range * std::cos(rx_angle);
  }
  detctionGraph->setData(x, y);
  QPen pen;
  pen.setColor(QColor(QColorConstants::Green));
  ui->plot->graph()->setPen(pen);
  ui->plot->graph()->setLineStyle((QCPGraph::LineStyle::lsNone));
  ui->plot->graph()->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssPlus, 4));

  ui->plot->replot();
}

void WaterColumnView::spinOnce(){
  if(rclcpp::ok()){
    rclcpp::spin_some(node_);
  }
  else
    QApplication::quit();
}

void WaterColumnView::on_wc_topic_currentIndexChanged(const QString &arg1)
{
  if(arg1.toStdString()==""){
    return;
  }
  if(!wc_sub_ || wc_sub_->get_topic_name() != arg1.toStdString()){
    using std::placeholders::_1;
    wc_sub_ = node_->create_subscription<marine_acoustic_msgs::msg::RawSonarImage>(
        arg1.toStdString(), 1, std::bind(&WaterColumnView::wcCallback, this, _1));;
    new_msg = true;
  }
}

void WaterColumnView::updateRangeBearing(QMouseEvent *event){
  QPoint p = event->pos();
  double x = ui->plot->xAxis->pixelToCoord(p.x());
  double y = ui->plot->yAxis->pixelToCoord(p.y());
  double range = std::sqrt(x*x + y*y);
  double bearing = std::atan2(x,y) * 180.0 / kPI; // fix pi constant
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
  auto master_topics  = this->get_topic_names_and_types();

  {
    ui->wc_topic->clear();
    QStringList topic_list;
    for(auto topic : master_topics){
      QString::fromStdString("topic[0]");
      if(topic.second[0]=="marine_acoustic_msgs/msg/RawSonarImage"){
        QString::fromStdString(topic.first);
        topic_list.push_back(QString::fromStdString(topic.first));
      }
    }
    ui->wc_topic->addItems(topic_list);
  }
  {
    ui->detect_topic->clear();
    QStringList topic_list;
    for(auto topic : master_topics){
      QString::fromStdString("topic[0]");
      if(topic.second[0]=="marine_acoustic_msgs/msg/SonarDetections" ||
          topic.second[0]=="marine_acoustic_msgs/msg/SonarRanges"){
        QString::fromStdString(topic.first);
        topic_list.push_back(QString::fromStdString(topic.first));
        topic_to_type[topic.first] = topic.second[0];
      }
    }
    ui->detect_topic->addItems(topic_list);
  }
  return;
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

void WaterColumnView::on_detect_topic_currentTextChanged(const QString &arg1)
{
  std::string topic = arg1.toStdString();
  if(topic==""){
    return;
  }
  if(!det_sub_ || det_sub_->get_topic_name() != topic){
    using std::placeholders::_1;

    auto const tt = topic_to_type.find(topic);

    if(tt == topic_to_type.end())
      return;

    if(tt->second == "marine_acoustic_msgs/msg/SonarDetections")
    {
      rng_sub_.reset();
      det_sub_ = node_->create_subscription<marine_acoustic_msgs::msg::SonarDetections>(
          topic, 1, std::bind(&WaterColumnView::detectionCallback, this, _1));
      new_msg = true;
    }
    else if (tt->second == "marine_acoustic_msgs/msg/SonarRanges")
    {
      det_sub_.reset();
      rng_sub_ = node_->create_subscription<marine_acoustic_msgs::msg::SonarRanges>(
          topic, 1, std::bind(&WaterColumnView::rangesCallback, this, _1));
      new_msg = true;
    }
  }
}

void WaterColumnView::on_color_ramp_select_currentIndexChanged(int index)
{
  switch (index) {
  case 0:
    colorMap->setGradient(QCPColorGradient::gpNight);
    break;
  case 1:
    colorMap->setGradient(QCPColorGradient::gpCold);
    break;
  case 2:
    colorMap->setGradient(QCPColorGradient::gpHot);
    break;
  case 3:
    colorMap->setGradient(QCPColorGradient::gpGrayscale);
    break;
  default:
    break;
  }
}
