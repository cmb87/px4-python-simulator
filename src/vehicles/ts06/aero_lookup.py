#!/usr/bin/env python3
"""
Aerodynamic Lookup Table Module
Loads precompiled aerodynamic coefficients from a consolidated CSV table
and provides fast multidimensional linear interpolation for flight simulation.
Purely function-based implementation.
"""

from __future__ import annotations

import csv
from pathlib import Path
import numpy as np


# Global variables to store the loaded grid data (loaded lazily)
_GRID_SPEEDS: list[float] | None = None
_GRID_COEFFS: dict[float, dict] | None = None


def _load_table():
    """Loads compiled coefficients from CSV and builds interpolation grids, filling any holes."""
    global _GRID_SPEEDS, _GRID_COEFFS
    
    # Locate the CSV file
    csv_path = None
    current_dir = Path(__file__).resolve().parent
    search_paths = [
        current_dir / "aerodynamics_table.csv",
        current_dir.parent / "aerodynamics_table.csv",
        Path("/home/cpeeren/projects/03_airframes/openfoam_cfd/aerodynamics_table.csv"),
    ]
    for path in search_paths:
        if path.is_file():
            csv_path = path
            break
            
    if csv_path is None:
        raise FileNotFoundError("Could not automatically locate 'aerodynamics_table.csv'.")
        
    raw_data = []
    with csv_path.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)

        for row in reader:
            u_val = float(row["u"])
            alpha_val = float(row["alpha"])
            beta_val = float(row["beta"])
            coeffs = [
                float(row["Cd"]),
                float(row["Cs"]),
                float(row["Cl"]),
                float(row["CmRoll"]),
                float(row["CmPitch"]),
                float(row["CmYaw"]),
            ]
            raw_data.append((u_val, alpha_val, beta_val, coeffs))
            
    if not raw_data:
        raise ValueError(f"No valid aerodynamic records found in {csv_path}")
        
    # Group by speed U
    by_u = {}
    for u_val, alpha_val, beta_val, coeffs in raw_data:
        if u_val not in by_u:
            by_u[u_val] = []
        by_u[u_val].append((alpha_val, beta_val, coeffs))
        
    _GRID_SPEEDS = sorted(by_u.keys())
    _GRID_COEFFS = {}
    
    for u_val, points in by_u.items():
        alphas = sorted(list(set(p[0] for p in points)))
        betas = sorted(list(set(p[1] for p in points)))
        
        # Create the 2D grid: (len(alphas), len(betas), 6)
        grid = np.full((len(alphas), len(betas), 6), np.nan)
        
        # Populate grid with available cases
        for alpha_val, beta_val, coeffs in points:
            i = alphas.index(alpha_val)
            j = betas.index(beta_val)
            grid[i, j] = coeffs
            
        # Linearly interpolate or extrapolate missing grid cells along the alpha direction
        for j in range(len(betas)):
            valid_indices = [i for i in range(len(alphas)) if not np.isnan(grid[i, j, 0])]
            if not valid_indices:
                continue
                
            if len(valid_indices) < len(alphas):
                x_data = [alphas[i] for i in valid_indices]
                for k in range(6):
                    y_data = [grid[i, j, k] for i in valid_indices]
                    for i in range(len(alphas)):
                        if np.isnan(grid[i, j, k]):
                            grid[i, j, k] = _interp1d_extrap(alphas[i], x_data, y_data)
                            
        _GRID_COEFFS[u_val] = {
            "alphas": alphas,
            "betas": betas,
            "grid": grid,
        }


def _interp1d_extrap(x_query: float, x_data: list[float], y_data: list[float]) -> float:
    """Performs 1D linear interpolation or extrapolation."""
    if len(x_data) == 1:
        return y_data[0]
        
    if x_data[0] <= x_query <= x_data[-1]:
        for idx in range(len(x_data) - 1):
            if x_data[idx] <= x_query <= x_data[idx+1]:
                x0, x1 = x_data[idx], x_data[idx+1]
                y0, y1 = y_data[idx], y_data[idx+1]
                t = (x_query - x0) / (x1 - x0) if x1 != x0 else 0.0
                return y0 + t * (y1 - y0)
    elif x_query < x_data[0]:
        x0, x1 = x_data[0], x_data[1]
        y0, y1 = y_data[0], y_data[1]
        t = (x_query - x0) / (x1 - x0)
        return y0 + t * (y1 - y0)
    else:
        x0, x1 = x_data[-2], x_data[-1]
        y0, y1 = y_data[-2], y_data[-1]
        t = (x_query - x0) / (x1 - x0)
        return y0 + t * (y1 - y0)
    return 0.0


def _interpolate_2d(u_val: float, alpha: float, beta: float) -> list[float]:
    """Performs 2D bilinear interpolation within a specific speed grid."""
    grid_data = _GRID_COEFFS[u_val]
    alphas = grid_data["alphas"]
    betas = grid_data["betas"]
    grid = grid_data["grid"]
    
    # Clamp inputs to grid boundaries
    alpha_clamped = max(alphas[0], min(alphas[-1], alpha))
    beta_clamped = max(betas[0], min(betas[-1], beta))
    
    # Find alpha interval [i, i+1]
    i = 0
    if len(alphas) > 1:
        for r in range(len(alphas) - 1):
            if alphas[r] <= alpha_clamped <= alphas[r+1]:
                i = r
                break
                
    # Find beta interval [j, j+1]
    j = 0
    if len(betas) > 1:
        for c in range(len(betas) - 1):
            if betas[c] <= beta_clamped <= betas[c+1]:
                j = c
                break
                
    # Weights
    t_a = (alpha_clamped - alphas[i]) / (alphas[i+1] - alphas[i]) if len(alphas) > 1 and alphas[i+1] != alphas[i] else 0.0
    t_b = (beta_clamped - betas[j]) / (betas[j+1] - betas[j]) if len(betas) > 1 and betas[j+1] != betas[j] else 0.0
    
    # Bilinear interpolation for each coefficient
    coeffs = []
    for k in range(6):
        v00 = grid[i, j, k]
        v10 = grid[i+1, j, k] if len(alphas) > 1 else v00
        v01 = grid[i, j+1, k] if len(betas) > 1 else v00
        v11 = grid[i+1, j+1, k] if (len(alphas) > 1 and len(betas) > 1) else v01 if len(betas) > 1 else v10
        
        v_interp = (1 - t_a) * (1 - t_b) * v00 + t_a * (1 - t_b) * v10 + (1 - t_a) * t_b * v01 + t_a * t_b * v11
        coeffs.append(v_interp)
        
    return coeffs


def lookup_aerodynamics(u: float, alpha: float, beta: float) -> tuple[float, float, float, float, float, float]:
    """
    Look up the aerodynamic coefficients (Cd, Cs, Cl, CmRoll, CmPitch, CmYaw)
    for a given u, alpha, and beta.
    
    Symmetry is applied to negative beta values.
    Out-of-bounds alpha and beta are clamped to the grid boundaries.
    
    Parameters:
        u (float): Freestream velocity [m/s]
        alpha (float): Angle of attack [deg]
        beta (float): Sideslip angle [deg]
        
    Returns:
        tuple: (Cd, Cs, Cl, CmRoll, CmPitch, CmYaw)
            - Cd: Drag force coefficient (X-axis)
            - Cs: Side force coefficient (Y-axis)
            - Cl: Lift force coefficient (Z-axis)
            - CmRoll: Roll moment coefficient (X-axis)
            - CmPitch: Pitch moment coefficient (Y-axis)
            - CmYaw: Yaw moment coefficient (Z-axis)
    """
    global _GRID_SPEEDS, _GRID_COEFFS
    if _GRID_COEFFS is None:
        _load_table()
        
    # 1. Apply symmetry on beta
    beta_sign = -1.0 if beta < 0.0 else (0.0 if abs(beta) < 1e-9 else 1.0)
    beta_query = abs(beta)
    
    if not _GRID_SPEEDS:
        raise ValueError("No grid speeds available.")
        
    # Clamp speed U to grid bounds
    u_clamped = max(_GRID_SPEEDS[0], min(_GRID_SPEEDS[-1], u))
    
    if len(_GRID_SPEEDS) == 1:
        coeffs = _interpolate_2d(_GRID_SPEEDS[0], alpha, beta_query)
    else:
        # Find interval for U
        idx = 0
        for i in range(len(_GRID_SPEEDS) - 1):
            if _GRID_SPEEDS[i] <= u_clamped <= _GRID_SPEEDS[i+1]:
                idx = i
                break
                
        u0, u1 = _GRID_SPEEDS[idx], _GRID_SPEEDS[idx+1]
        coeffs0 = _interpolate_2d(u0, alpha, beta_query)
        coeffs1 = _interpolate_2d(u1, alpha, beta_query)
        
        t_u = (u_clamped - u0) / (u1 - u0) if u1 != u0 else 0.0
        coeffs = [c0 + t_u * (c1 - c0) for c0, c1 in zip(coeffs0, coeffs1)]
        
    # 2. Apply aerodynamic symmetry transformations for negative beta:
    # Symmetric: Cd (0), Cl (2), CmPitch (4)
    # Anti-symmetric: Cs (1), CmRoll (3), CmYaw (5)
    cd = coeffs[0]
    cs = coeffs[1] * beta_sign
    cl = coeffs[2]
    cm_roll = coeffs[3] * beta_sign
    cm_pitch = coeffs[4]
    cm_yaw = coeffs[5] * beta_sign
    
    return (cd, cs, cl, cm_roll, cm_pitch, cm_yaw)
