// struct to store the sensors data
#ifndef SENSORS_DATA_H
#define SENSORS_DATA_H

typedef struct {
  int sensor_id;
  int sensor_value;
  int sensor_relay;
} SensorData;

#define MAX_SENSORS 4

#endif // SENSORS_DATA_H