//
// Created by Juha-Matti Tirila on 02/10/2017.
//

#ifndef LIDAR_RADAR_KALMAN_CPP_MEASUREMENT_PACKAGE_H
#define LIDAR_RADAR_KALMAN_CPP_MEASUREMENT_PACKAGE_H

#include "Dense"

class MeasurementPackage {
public:
  int64_t timestamp_;

  enum SensorType {
    LASER, RADAR
  } sensor_type_;

  Eigen::VectorXd raw_measurements_;

};


#endif //LIDAR_RADAR_KALMAN_CPP_MEASUREMENT_PACKAGE_H
