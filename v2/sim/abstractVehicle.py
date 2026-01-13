
import matplotlib.pyplot as plt
import numpy as np
from quaternion import Quaternion
import sys
import time

from parameters import Parameters
from forces import forces, railForces
from sensors import sensors
from dynamics import dynamics, railDynamics


class AbstractVehicle():
    def __init__(self,):

        self.P = Parameters()  # Load parameters, assuming this is defined in parameters.py

        railPitch = np.deg2rad(0.0)
        railYaw = np.deg2rad(0.0)

        quatRail = Quaternion.euler2quat(np.asarray([0.0,railPitch,railYaw]))
        self.P.rail_dir_ned = Quaternion.Mfg(quatRail).T @ np.asarray([1,0,0])

        # Example usage
        self.t = 0.0
        self.y = np.zeros(13)  # Initial state

        self.y[7] = 0.0 # Start speed in body dir
        self.y[0:3] = self.P.rail_start_ned
        self.y[3:7] = quatRail

        self.ydot = np.zeros_like(self.y)

        self.u = np.zeros(4)  # Example control inputs
        self.u[3] = 0.78
        self.u[1] = 0.00

        self.armed = False

        self.wind = np.zeros(6)  # Example wind vector
        self.dt= 10000.0 / 1e6

        self.sensors = {}

        self.sim_current_us = time.time_ns() // 1000

    # --------------------------------------------------
    def step(self):

        if self.armed:
            tau = forces(self.t, self.y, self.u, self.wind, self.P)
        else:
            tau = np.zeros(6)

        if self.P.left_rail:
            ydot = dynamics(self.t, self.self.y, self.P, tau)
            
        else:
            tauRail = railForces(self.t, self.y, self.u, self.wind, self.P)
            self.ydot = railDynamics(self.t, self.y, self.P, tau+tauRail)
        

        self.sensors = sensors(self.t, self.y, self.ydot, self.u, self.wind, self.P, self.dt)


        self.y += self.ydot * self.dt  # Update state using Euler method
        self.t += self.dt

        self.sim_current_us += self.dt*1e6

        #print(f"Time: {t:.2f}, State: {np.around(y,2)}")


    def get_sim_time_ms(self):
        return int(self.sim_current_us/1000.0)



if __name__ == '__main__':

    vehicle = AbstractVehicle()
    vehicle.armed = True

    for i in range(10):
        print(vehicle.sim_current_us, np.around(vehicle.y,2))
        vehicle.step()



