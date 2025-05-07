#ifndef SENSOR_FILTER_H
#define SENSOR_FILTER_H
 
 //ensure you #include "SensorFilter.h"
 //to include in main - define e.g. SensorFilter filter(0.01, 5, 0, 1);
 // // Apply Kalman filter - float filteredDistance = fliter.update(distance);
 
 
 class SensorFilter {
   private:
     float dist_est; // Estimated state
     float error_est; // Estimated error covariance
     float Q;     // Process noise covariance
     float R;     // Measurement noise covariance
     float coefficent;
     float exponent;
     int pin_num;
 
   public:
     SensorFilter(float processNoise, float measurementNoise, float initialEstimate, float initialError, float co, float exp, int pin) {
         Q = processNoise;
         R = measurementNoise;
         dist_est = initialEstimate;
         error_est = initialError;
         coefficent = co;
         exponent = exp;
         pin_num = pin;
     }
 
     float update(float raw_distance) {
         // Find predicted variables
         float dist_pred = dist_est;
         float error_pred = error_est + Q;
 
         // Update distance
         float K = error_pred / (error_pred + R);
         dist_est = dist_pred + K * (raw_distance - dist_pred);
         error_est = (1 - K) * error_pred;
 
         return dist_est;
     }
 
     float read(){
       int rawValue = analogRead(pin_num);
     
       float distance = coefficent * pow(rawValue, exponent);
 
       if (distance < 4) distance = 4;
       if (distance > 1000) distance = 1000;
 
       // Apply Kalman filter
       float filteredDistance = update(distance);
       return filteredDistance;
     }
 
 
 
 };
 
 #endif
