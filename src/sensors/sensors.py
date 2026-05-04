import numpy as np

from vehicles.base_component import SimComponentBase
from dynamics.quaternion import Quaternion
from .barometer import BarometerSensor
from .gps import GpsSensor
from .imu import SimpleIMU
from .magnetometer import MagnetometerSim
from .airspeed import AirspeedSensor


DEFAULT_GPS_ORIGIN = {
    "lat": 0.0,
    "lon": 0.0,
    "alt": 0.0,
}


class SensorSuite(SimComponentBase):
    def __init__(self):
        super().__init__()
        self.mag = MagnetometerSim()
        self.mag.set_noise(True)
        self.imu = SimpleIMU()
        self.imu.set_noise(True)

        self._sensor_params_initialized = False

        self.gps = GpsSensor()
        self.gps.set_noise(True)
        self.gps.set_update_rate(5.0)

        self.baro = BarometerSensor()
        self.baro.set_update_rate(20.0)
        self.baro.set_drift_rate(0.05)
        self.baro.set_noise(True)

        self.airspeed = AirspeedSensor()

    def _initialize_from_parameters(self, P):
        gps_origin = getattr(P, "gps_origin", DEFAULT_GPS_ORIGIN)
        lat_deg = float(gps_origin.get("lat", DEFAULT_GPS_ORIGIN["lat"]))
        lon_deg = float(gps_origin.get("lon", DEFAULT_GPS_ORIGIN["lon"]))
        alt_m = float(gps_origin.get("alt", DEFAULT_GPS_ORIGIN["alt"]))
        gravity_mps2 = float(getattr(P, "gravity", 9.81))

        self.gps.set_home(lat_deg, lon_deg, alt_m)
        self.baro.set_home_altitude(alt_m)
        self.baro.set_gravity(gravity_mps2)
        self.imu.set_gravity(gravity_mps2)

        self._sensor_params_initialized = True

    def update(self, t_us, paused):
        if paused:
            return self.last_output

        y = self._inputs.get("y")
        ydot = self._inputs.get("ydot")
        wind = self._inputs.get("wind")
        P = self._inputs.get("P")

        if y is None or ydot is None or P is None:
            raise ValueError("SensorSuite requires inputs: y, ydot, P")

        if not self._sensor_params_initialized:
            self._initialize_from_parameters(P)

        # Update all sensors with common inputs
        self.imu.set_inputs(y=y, ydot=ydot, P=P)
        acc_meas, gyro_meas = self.imu.update(t_us, paused=False)

        self.mag.set_inputs(y=y, mag_field_ned=P.magnetic_ned)
        mag_meas = self.mag.update(t_us, paused=False)["mag_field_body_gauss"]

        self.baro.set_inputs(y=y)
        baro_reading = self.baro.update(t_us, paused=False)
        
        self.airspeed.set_inputs(y=y, wind=wind, P=P)
        airspeed_reading = self.airspeed.update(t_us, paused=False)

        self.gps.set_inputs(y=y)
        gps_data = self.gps.update(t_us, paused=False)
        
        # Format outputs as expected by the caller
        static_pressure = baro_reading["absolute_pressure_hpa"] * 100.0
        baro_meas = {
            "staticAbsolute": static_pressure,
            "static": static_pressure,
            "dynamic": airspeed_reading["dynamic_pressure_pa"],
            "pressure_altitude_m": float(baro_reading["pressure_altitude_m"]),
        }

        gps_meas = np.array([
            gps_data["latitude_deg"],
            gps_data["longitude_deg"],
            gps_data["altitude_m"],
            gps_data["velocity_north"],
            gps_data["velocity_east"],
            gps_data["velocity_up"],
            0.0,
        ])

        quat = y[3:7] / np.linalg.norm(y[3:7])
        euler = np.rad2deg(Quaternion.quat2Euler(quat))

        self.last_output = {
            "accelerometer": acc_meas,
            "gyroscope": gyro_meas,
            "magnetometer": mag_meas,
            "barometer": baro_meas,
            "airspeed_ias_mps": airspeed_reading["airspeed_ias_mps"],
            "airspeed_tas_mps": airspeed_reading["airspeed_tas_mps"],
            "gps": gps_meas,
            "imu_updated": self.imu.is_updated(),
            "mag_updated": self.mag.is_updated(),
            "baro_updated": self.baro.is_updated(),
            "diff_press_updated": self.airspeed.is_updated(),
            "gps_updated": self.gps.is_updated(),
            "euler": euler,
        }
        
        # Reset update flags in children
        self.imu.updated = False
        self.mag.updated = False
        self.baro.updated = False
        self.gps.updated = False
        self.airspeed.updated = False

        self._last_t_us = int(t_us)
        return self.last_output


_DEFAULT_SENSOR_SUITE = SensorSuite()


def sensors(t, y, ydot, u, wind, P, dt):
    _DEFAULT_SENSOR_SUITE.set_inputs(y=y, ydot=ydot, u=u, wind=wind, P=P)
    return _DEFAULT_SENSOR_SUITE.update(int(float(t) * 1e6), paused=False)
