#include <tracker/Tracklet.h>

Tracklet::Tracklet(int idTrack, double x, double y)
: loss_count_(0), length(0), id_(idTrack) {
  // set id
  // set loss count to 0

  // initialize filter
  kf_.init(0.1);
  kf_.setState(x, y);
}

double dist(double x1, double y1, double x2, double y2) {
  return sqrt(pow(x1-x2, 2) + pow(y1-y2, 2));
}

// struct TraceLen {
//   Tracklet *t;
//   double prev_x, prev_y;

//   double dist(double x1, double y1, double x2, double y2) {
//     return sqrt(pow(x1-x2, 2) + pow(y1-y2, 2));
//   }

//   TraceLen(Tracklet *t) : t(t) {
//     prev_x = t->getX();
//     prev_y = t->getY();
//   }

//   ~TraceLen() {
//     // std::cout << prev_x << t->getX() << std::endl;
//     t->length += dist(prev_x, prev_y, t->getX(), t->getY());
//   }
// };

// Predict a single measurement
void Tracklet::predict() {
  // TraceLen(this);
  auto prev_x = getX();
  auto prev_y = getY();

  kf_.predict();
  loss_count_++;

  // By thresholding distance avoid accumulation of small centemeters from tracklets standing still
  auto d =  dist(prev_x, prev_y, getX(), getY());
  if (d > 0.1) length += dist(prev_x, prev_y, getX(), getY());
}

// Update with a real measurement
void Tracklet::update(double x, double y, bool lidarStatus) {
  // TraceLen(this);
  auto prev_x = getX();
  auto prev_y = getY();

  Eigen::VectorXd raw_measurements_ = Eigen::VectorXd(2);

  // measurement update
  if (lidarStatus)
  {
    raw_measurements_ << x, y;
    kf_.update(raw_measurements_);
    loss_count_ = 0;
  }

  // By thresholding distance avoid accumulation of small centemeters from tracklets standing still
  auto d =  dist(prev_x, prev_y, getX(), getY());
  if (d > 0.1) length += dist(prev_x, prev_y, getX(), getY());
}
