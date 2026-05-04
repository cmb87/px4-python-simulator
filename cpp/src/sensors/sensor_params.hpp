#pragma once

// Base Frequencies (Hz)
#define SIM_RATE_HZ 250.0
#define WORK_RATE_HZ 125.0
#define DT_DEFAULT (1.0 / WORK_RATE_HZ)

// Sensor Update Rates (Hz)
#define IMU_UPDATE_RATE_HZ 125.0
#define MAG_UPDATE_RATE_HZ 100.0
#define BARO_UPDATE_RATE_HZ 20.0
#define GPS_UPDATE_RATE_HZ 5.0
#define AIRSPEED_UPDATE_RATE_HZ 125.0

// IMU Noise Parameters (SimpleIMU)
#define IMU_ACCEL_STD 0.0001
#define IMU_GYRO_STD 0.00001

// GPS Noise Parameters
#define GPS_XY_NOISE_DENSITY 2.0e-4
#define GPS_Z_NOISE_DENSITY 4.0e-4
#define GPS_VXY_NOISE_DENSITY 0.2
#define GPS_VZ_NOISE_DENSITY 0.004
#define GPS_XY_RANDOM_WALK 2.0
#define GPS_Z_RANDOM_WALK 4.0

// Magnetometer Parameters
#define MAG_NOISE_DENSITY 0.4e-3
#define MAG_RANDOM_WALK 6.4e-6
#define MAG_BIAS_CORRELATION_TIME 600.0

// Barometer Parameters
#define BARO_DRIFT_PA_S 0.05
#define BARO_NOISE_STD 1.0

// Airspeed Parameters
#define AIRSPEED_LPF_TAU 0.08
#define AIRSPEED_NOISE_STD 0.002

// Physical Constants
#define R_EARTH 6371000.0
#define ISA_TEMPERATURE_MSL_K 288.15
#define ISA_PRESSURE_MSL_PA 101325.0
#define ISA_LAPSE_RATE_K_PER_M 0.0065
#define ISA_AIR_DENSITY_MSL_KGPM3 1.225
#define ABSOLUTE_ZERO_C -273.15
