//
// Created by Juha-Matti Tirila on 02/10/2017.
//


#ifndef LIDAR_RADAR_KALMAN_CPP_TRACKING_H
#define LIDAR_RADAR_KALMAN_CPP_TRACKING_H

#include "measurement_package.h"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"

class Tracking {
public:
  Tracking();
  virtual ~Tracking();
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);
  KalmanFilter kf_;

private:
  bool is_initialized_;
  int64_t previous_timestamp_;

  //acceleration noise components
  float noise_ax;
  float noise_ay;

};


#endif //LIDAR_RADAR_KALMAN_CPP_TRACKING_H
