#!/usr/bin/env python3
"""
Plotting script for TS06 fixed-wing multi-motor tailsitter.
This script solves for the steady-state cruise trim at a given airspeed,
and then plots the vehicle from the X-Z (side) and Y-Z (rear) planes,
illustrating all the acting physical forces (gravity, lift, drag, and thrusts).
"""

import sys
import os
import argparse
import numpy as np
import matplotlib.pyplot as plt

# Ensure 'src' is in sys.path
current_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.abspath(os.path.join(current_dir, "..", ".."))
if src_dir not in sys.path:
    sys.path.insert(0, src_dir)

from vehicles.ts06.forces import forces
from vehicles.ts06.parameters import Ts06Parameters
from dynamics.quaternion import Quaternion
from vehicles.ts06.trim import solve_symmetric_trim, solve_general_trim


def get_vehicle_points_body(P):
    """
    Returns coordinate points in the body frame for drawing the schematic of the TS06.
    """
    # 1. Fuselage box (length 0.8m, width 0.08m, height 0.08m)
    # x in [-0.4, 0.4], y in [-0.04, 0.04], z in [-0.04, 0.04]
    fuse_x = np.array([-0.4, 0.4, 0.4, -0.4, -0.4])
    fuse_z = np.array([-0.04, -0.04, 0.04, 0.04, -0.04])
    fuse_y = np.array([-0.04, -0.04, 0.04, 0.04, -0.04])

    # 2. Wing chord line (chord 0.2m) at x = x_wing, z = z_wing
    wing_x = np.array([P.x_wing - 0.1, P.x_wing + 0.1])
    wing_z = np.array([P.z_wing, P.z_wing])
    
    # 3. Propeller disk segments in body frame
    # Diameter is approx 0.2m, rotated perpendicular to X axis
    props_x = []
    props_z = []
    for i in range(4):
        p_x = np.array([P.x_motors[i], P.x_motors[i]])
        p_z = np.array([P.z_motors[i] - 0.1, P.z_motors[i] + 0.1])
        props_x.append(p_x)
        props_z.append(p_z)

    return {
        'fuse_x': fuse_x, 'fuse_z': fuse_z, 'fuse_y': fuse_y,
        'wing_x': wing_x, 'wing_z': wing_z,
        'props_x': props_x, 'props_z': props_z
    }


def rotate_xz(x, z, theta):
    """
    Rotates 2D coordinates in X-Z plane by angle theta (nose up).
    In our convention:
    - X_plot = x * cos(theta) + z * sin(theta)
    - Z_plot = x * sin(theta) - z * cos(theta)
    """
    x_rot = x * np.cos(theta) + z * np.sin(theta)
    z_rot = x * np.sin(theta) - z * np.cos(theta)
    return x_rot, z_rot


def main():
    parser = argparse.ArgumentParser(description="Plot TS06 trimmed vehicle state and force vectors.")
    parser.add_argument("--velocity", type=float, default=45.0,
                        help="Target cruise velocity in m/s (default: 18.0)")
    parser.add_argument("--general", action="store_true",
                        help="Use general 5-variable trim solver instead of symmetric solver")
    parser.add_argument("--output", type=str, default="ts06_trim_forces.png",
                        help="File path to save the generated plot image (default: ts06_trim_forces.png)")
    args = parser.parse_args()

    P = Ts06Parameters()

    # 1. Solve the trim problem to find theta and u
    if args.general:
        print(f"Solving general 5-variable trim at {args.velocity} m/s...")
        sol, fun, _ = solve_general_trim(args.velocity, P)
        theta = sol[0]
        u = sol[1:5]
    else:
        print(f"Solving symmetric 3-variable trim at {args.velocity} m/s...")
        sol, fun, _ = solve_symmetric_trim(args.velocity, P)
        theta = sol[0]
        u = np.array([sol[1], sol[2], sol[1], sol[2]])

    print(f"Trim solved: theta = {np.rad2deg(theta):.4f} deg, throttles = {list(np.round(u, 4))}")

    # Re-evaluate forces to get individual components
    y = np.zeros(13)
    q = Quaternion.euler2quat([0.0, theta, 0.0])
    y[3:7] = q
    
    mfg = Quaternion.Mfg(q)
    mgf = mfg.T
    vel_ned = np.array([args.velocity, 0.0, 0.0])
    vel_body = mfg @ vel_ned
    y[7:10] = vel_body
    y[10:13] = np.zeros(3)
    
    wind = np.zeros(6)
    
    # Aerodynamic-only forces (motors off)
    tau_aero = forces(0.0, y, np.zeros(4), wind, P)
    f_aero_body = tau_aero[0:3]
    
    f_aero_ned = mgf @ f_aero_body
    lift_force = -f_aero_ned[2]
    drag_force = -f_aero_ned[0]

    # Individual motor thrusts
    thrusts = []
    for i in range(4):
        r_motor = np.array([P.x_motors[i], P.y_motors[i], P.z_motors[i]], dtype=float)
        v_motor_body = vel_body
        v_a_motor = v_motor_body[0]
        throttle_i = np.clip(u[i], 0.0, 1.0)
        v_d_motor = v_a_motor + throttle_i * (P.k_motor - v_a_motor)
        thrust_i = 0.5 * P.rho * P.S_prop * P.C_prop * v_d_motor * (v_d_motor - v_a_motor)
        thrusts.append(thrust_i)

    # 2. Set up the plotting window
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 7.5))
    fig.suptitle(f"TS06 Force and Moment Balance at Cruise Speed = {args.velocity:.1f} m/s\n"
                 f"Trim Pitch Angle (θ) = {np.rad2deg(theta):.2f}° | L/D = {lift_force/drag_force if drag_force != 0 else 0:.2f}",
                 fontsize=14, fontweight='bold')

    # Get body-frame geometry
    geom = get_vehicle_points_body(P)

    # ------------------------------------------------------------
    # LEFT SUBPLOT: X-Z PLANE (Side View)
    # ------------------------------------------------------------
    ax1.set_title("X-Z Plane (Side View)\nLooking from the Right Side (-Y direction)", fontsize=12)
    ax1.set_xlabel("Inertial X_ned (Forward) [meters]", fontsize=10)
    ax1.set_ylabel("Inertial -Z_ned (Altitude) [meters]", fontsize=10)

    # Plot ground reference line
    ax1.axhline(0, color='gray', linestyle='--', linewidth=0.8)

    # Rotate and plot Fuselage
    fx_rot, fz_rot = rotate_xz(geom['fuse_x'], geom['fuse_z'], theta)
    ax1.plot(fx_rot, fz_rot, 'k-', linewidth=2, label="Fuselage")
    ax1.fill(fx_rot, fz_rot, color='lightgray', alpha=0.5)

    # Rotate and plot Wing chord
    wx_rot, wz_rot = rotate_xz(geom['wing_x'], geom['wing_z'], theta)
    ax1.plot(wx_rot, wz_rot, 'b-', linewidth=4, label="Wing Chord")

    # Rotate and plot Propeller disks
    for i in range(4):
        px_rot, pz_rot = rotate_xz(geom['props_x'][i], geom['props_z'][i], theta)
        # Use different colors for top/bottom motors to make it readable
        color = 'darkorange' if i in [0, 2] else 'forestgreen'
        label = "Top Prop Disks" if i == 0 else ("Bottom Prop Disks" if i == 1 else "")
        ax1.plot(px_rot, pz_rot, color=color, linestyle='-', linewidth=2, label=label)

    # Add Center of Gravity marker
    ax1.plot(0, 0, 'ro', markersize=8, label="Center of Gravity (CG)")

    # Force Vectors (X-Z plane)
    # Scale force vectors for neat visualization
    scale = 0.015  # 1 Newton = 0.015 meters on plot

    # Gravity Force vector at CG
    gravity_val = P.mass * P.gravity
    ax1.quiver(0, 0, 0, -gravity_val * scale, scale=1, scale_units='xy', angles='xy',
               color='red', width=0.008, zorder=5, label=f"Gravity ({gravity_val:.1f} N)")

    # Aerodynamic forces at Wing aerodynamic center (x_wing, z_wing)
    wx_ac_rot, wz_ac_rot = rotate_xz(P.x_wing, P.z_wing, theta)
    # Lift (points along -Z_ned, i.e., upward on plot)
    ax1.quiver(wx_ac_rot, wz_ac_rot, 0, lift_force * scale, scale=1, scale_units='xy', angles='xy',
               color='blue', width=0.008, zorder=5, label=f"Lift ({lift_force:.1f} N)")
    # Drag (points along -X_ned, i.e., leftward on plot)
    ax1.quiver(wx_ac_rot, wz_ac_rot, -drag_force * scale, 0, scale=1, scale_units='xy', angles='xy',
               color='teal', width=0.008, zorder=5, label=f"Aerodynamic Drag ({drag_force:.1f} N)")

    # Motor Thrust vectors at motor positions
    for i in range(4):
        mx_rot, mz_rot = rotate_xz(P.x_motors[i], P.z_motors[i], theta)
        # Thrust points along body +X axis, which rotates to:
        # tx_plot = T * cos(theta), tz_plot = T * sin(theta)
        tx_plot = thrusts[i] * np.cos(theta) * scale
        tz_plot = thrusts[i] * np.sin(theta) * scale
        color = 'darkorange' if i in [0, 2] else 'forestgreen'
        ax1.quiver(mx_rot, mz_rot, tx_plot, tz_plot, scale=1, scale_units='xy', angles='xy',
                   color=color, width=0.006, zorder=5)

    # Plot velocity vector coming from front to illustrate airflow
    ax1.quiver(0.6, 0.2, -0.2, 0, scale=1, scale_units='xy', angles='xy',
               color='cyan', width=0.005, label=f"Airflow ({args.velocity:.1f} m/s)")

    ax1.set_aspect('equal', 'box')
    ax1.grid(True, which='both', linestyle=':', alpha=0.5)
    ax1.set_xlim(-0.6, 0.8)
    ax1.set_ylim(-0.6, 0.6)
    ax1.legend(loc='lower left', fontsize=9)

    # ------------------------------------------------------------
    # RIGHT SUBPLOT: Y-Z PLANE (Rear View)
    # ------------------------------------------------------------
    ax2.set_title("Y-Z Plane (Rear View)\nLooking from behind (+X in body frame)", fontsize=12)
    ax2.set_xlabel("Inertial Y_ned (Right) [meters]", fontsize=10)
    ax2.set_ylabel("Inertial -Z_ned (Altitude) [meters]", fontsize=10)

    # Plot wings (spans from -span_wing to +span_wing at z_wing)
    # Since phi = 0, they remain horizontal at -z_wing
    wing_z_plot = -P.z_wing
    ax2.plot([-P.span_wing, P.span_wing], [wing_z_plot, wing_z_plot], 'b-', linewidth=4, label="Wing")

    # Plot Fuselage box from rear (0.08m square)
    fuse_box_y = np.array([-0.04, 0.04, 0.04, -0.04, -0.04])
    fuse_box_z = np.array([-0.04, -0.04, 0.04, 0.04, -0.04])
    ax2.plot(fuse_box_y, fuse_box_z, 'k-', linewidth=2, label="Fuselage Profile")
    ax2.fill(fuse_box_y, fuse_box_z, color='lightgray', alpha=0.5)

    # Plot 4 motors and propeller disks as circles
    for i in range(4):
        my_plot = P.y_motors[i]
        # In body frame, motor z is z_motors[i]. Since we rotate by theta,
        # the motor NED position z component is:
        # z_ned_motor = -x_motor * sin(theta) + z_motor * cos(theta)
        # So on our vertical axis (-z_ned), it is:
        mz_plot = P.x_motors[i] * np.sin(theta) - P.z_motors[i] * np.cos(theta)
        
        color = 'darkorange' if i in [0, 2] else 'forestgreen'
        
        # Draw propeller disk circles (radius 0.1m)
        circle = plt.Circle((my_plot, mz_plot), 0.1, color=color, fill=False, linestyle='--', linewidth=1.2)
        ax2.add_patch(circle)
        
        # Fill circle according to throttle/thrust
        # Higher throttle = solid circle
        circle_fill = plt.Circle((my_plot, mz_plot), 0.02 + 0.08 * (u[i] ** 0.5), color=color, alpha=0.3)
        ax2.add_patch(circle_fill)
        
        ax2.plot(my_plot, mz_plot, 'o', color=color, markersize=6)
        ax2.text(my_plot + 0.02, mz_plot + 0.02, f"M{i}\nT={thrusts[i]:.1f}N", fontsize=8, fontweight='bold', color=color)

    ax2.plot(0, 0, 'ro', markersize=8, label="CG")

    # Force Vectors (Y-Z plane)
    # Gravity (downward)
    ax2.quiver(0, 0, 0, -gravity_val * scale, scale=1, scale_units='xy', angles='xy',
               color='red', width=0.008, zorder=5)

    # Lift (upward at wing AC center y=0)
    # Wing AC position on vertical axis:
    wz_ac_plot = P.x_wing * np.sin(theta) - P.z_wing * np.cos(theta)
    ax2.quiver(0, wz_ac_plot, 0, lift_force * scale, scale=1, scale_units='xy', angles='xy',
               color='blue', width=0.008, zorder=5)

    # Motor thrust vertical components (upward since thrust tilts up by theta)
    # Vertical thrust component = T * sin(theta)
    for i in range(4):
        my_plot = P.y_motors[i]
        mz_plot = P.x_motors[i] * np.sin(theta) - P.z_motors[i] * np.cos(theta)
        tz_comp = thrusts[i] * np.sin(theta) * scale
        color = 'darkorange' if i in [0, 2] else 'forestgreen'
        if tz_comp > 1e-4:
            ax2.quiver(my_plot, mz_plot, 0, tz_comp, scale=1, scale_units='xy', angles='xy',
                       color=color, width=0.006, zorder=5)

    ax2.set_aspect('equal', 'box')
    ax2.grid(True, which='both', linestyle=':', alpha=0.5)
    ax2.set_xlim(-0.7, 0.7)
    ax2.set_ylim(-0.6, 0.6)
    ax2.legend(loc='lower left', fontsize=9)

    plt.tight_layout()
    
    # Save the output image
    output_path = os.path.abspath(args.output)
    plt.savefig(output_path, dpi=300)
    print(f"\n[SUCCESS] Forces plot successfully created and saved to: {output_path}")


if __name__ == "__main__":
    main()
