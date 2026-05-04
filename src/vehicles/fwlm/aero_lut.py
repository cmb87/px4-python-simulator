#!/usr/bin/env python3
"""Aerodynamic lookup-table module for 6DoF simulation.

This module loads a coefficient table parameterized by
alpha, beta, delta21, delta22 and provides interpolated coefficients,
body forces, and body moments for each simulation step.

Example:
    lut = AeroLookupTable.from_csv("aero_lookup.csv", s_ref=0.48, b_ref=1.2, c_ref=0.35)
    coeffs = lut.eval_coeffs(
        v_air=120.0,
        alpha_deg=4.0,
        beta_deg=1.5,
        delta21_deg=8.0,
        delta22_deg=-6.0,
        h_m=1200.0,
    )
    f_b, m_b = lut.forces_moments(
        v_air=120.0,
        alpha_deg=4.0,
        beta_deg=1.5,
        delta21_deg=8.0,
        delta22_deg=-6.0,
        h_m=1200.0,
    )
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Literal

import numpy as np
import pandas as pd
from scipy.interpolate import RegularGridInterpolator

FORCE_COLS = ("cf_x", "cf_y", "cf_z")
MOMENT_COLS = ("cm_x", "cm_y", "cm_z")
COEFF_COLS = FORCE_COLS + MOMENT_COLS
OPTIONAL_DERIVATIVE_COLS = (
    "cf_xp",
    "cf_yp",
    "cf_yr",
    "cf_zq",
    "cf_zad",
    "cm_yad",
    "cm_zad",
    "cm_zr",
)
GRID_COLS = ("alpha", "beta", "delta21", "delta22")
REQUIRED_COLS = GRID_COLS + COEFF_COLS

OutOfBoundsMode = Literal["clamp", "raise", "extrapolate"]


def _isa_density(h_m: float) -> float:
    """Return ISA density [kg/m^3] for geometric altitude [m]."""
    h = max(float(h_m), 0.0)

    # ISA constants
    g0 = 9.80665
    r = 287.05287
    t0 = 288.15
    p0 = 101325.0
    lapse = -0.0065
    h_trop = 11000.0

    if h <= h_trop:
        t = t0 + lapse * h
        p = p0 * (t / t0) ** (-g0 / (lapse * r))
    else:
        t11 = t0 + lapse * h_trop
        p11 = p0 * (t11 / t0) ** (-g0 / (lapse * r))
        p = p11 * np.exp(-g0 * (h - h_trop) / (r * t11))
        t = t11

    return p / (r * t)


@dataclass
class AeroLookupTable:
    s_ref: float
    b_ref: float
    c_ref: float
    out_of_bounds: OutOfBoundsMode
    _axes: dict[str, np.ndarray]
    _interps: dict[str, RegularGridInterpolator]
    _mach_ref: float | None
    force_sign: tuple[float, float, float]
    moment_sign: tuple[float, float, float]

    @classmethod
    def from_csv(
        cls,
        csv_path: str | Path,
        s_ref: float,
        b_ref: float,
        c_ref: float,
        out_of_bounds: OutOfBoundsMode = "clamp",
        force_sign: tuple[float, float, float] = (1.0, 1.0, 1.0),
        moment_sign: tuple[float, float, float] = (1.0, 1.0, 1.0),
    ) -> "AeroLookupTable":
        if out_of_bounds not in {"clamp", "raise", "extrapolate"}:
            raise ValueError("out_of_bounds must be one of: clamp, raise, extrapolate")
        if len(force_sign) != 3 or len(moment_sign) != 3:
            raise ValueError("force_sign and moment_sign must each have 3 entries")

        df = pd.read_csv(csv_path)
        df = df.loc[:, ~df.columns.str.startswith("Unnamed")].copy()

        missing = [c for c in REQUIRED_COLS if c not in df.columns]
        if missing:
            raise ValueError(f"Missing required columns: {missing}")

        axes = {
            col: np.sort(np.asarray(df[col].dropna().unique(), dtype=float))
            for col in GRID_COLS
        }

        expected_size = int(np.prod([len(axes[c]) for c in GRID_COLS]))
        if len(df) != expected_size:
            raise ValueError(
                "Table is not a complete regular grid over "
                f"{GRID_COLS}. Expected {expected_size} rows, got {len(df)}."
            )

        multi_index = pd.MultiIndex.from_product(
            [axes[c] for c in GRID_COLS],
            names=list(GRID_COLS),
        )
        coeff_cols = list(COEFF_COLS) + [col for col in OPTIONAL_DERIVATIVE_COLS if col in df.columns]
        grid_df = df.set_index(list(GRID_COLS))[coeff_cols].sort_index().reindex(multi_index)

        if grid_df.isnull().any().any():
            raise ValueError("Table contains missing coefficient values on the regular grid.")

        points = tuple(axes[c] for c in GRID_COLS)
        interp_kwargs = {
            "bounds_error": out_of_bounds == "raise",
            "fill_value": None,
        }

        shape = tuple(len(axes[c]) for c in GRID_COLS)
        interps: dict[str, RegularGridInterpolator] = {}
        for coeff in coeff_cols:
            values = np.asarray(grid_df[coeff].to_numpy(dtype=float)).reshape(shape)
            interps[coeff] = RegularGridInterpolator(points=points, values=values, **interp_kwargs)

        mach_ref = float(df["mach"].iloc[0]) if "mach" in df.columns else None
        return cls(
            s_ref=float(s_ref),
            b_ref=float(b_ref),
            c_ref=float(c_ref),
            out_of_bounds=out_of_bounds,
            _axes=axes,
            _interps=interps,
            _mach_ref=mach_ref,
            force_sign=(
                float(force_sign[0]),
                float(force_sign[1]),
                float(force_sign[2]),
            ),
            moment_sign=(
                float(moment_sign[0]),
                float(moment_sign[1]),
                float(moment_sign[2]),
            ),
        )

    def _bound_point(self, point: np.ndarray) -> np.ndarray:
        if self.out_of_bounds != "clamp":
            return point
        bounded = point.copy()
        for i, col in enumerate(GRID_COLS):
            bounded[i] = np.clip(bounded[i], self._axes[col][0], self._axes[col][-1])
        return bounded

    def eval_coeffs(
        self,
        v_air: float,
        alpha_deg: float,
        beta_deg: float,
        delta21_deg: float,
        delta22_deg: float,
        rho: float | None = None,
        h_m: float = 0.0,
    ) -> dict[str, float]:
        """Return interpolated aerodynamic coefficients for current state.

        Notes:
        - `v_air` is accepted for 6DoF-call compatibility; coefficients are looked up
          from attitude and control deflections.
        - If your table has a single Mach level, airspeed influences loads via dynamic
          pressure only (in `forces_moments`).
        """
        _ = (v_air, rho, h_m)
        point = np.array([alpha_deg, beta_deg, delta21_deg, delta22_deg], dtype=float)
        point = self._bound_point(point)

        coeffs: dict[str, float] = {}
        for name, interp in self._interps.items():
            value = np.asarray(interp(point)).item()
            coeffs[name] = float(np.real(value))
        return coeffs

    def forces_moments(
        self,
        v_air: float,
        alpha_deg: float,
        beta_deg: float,
        delta21_deg: float,
        delta22_deg: float,
        rho: float | None = None,
        h_m: float = 0.0,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Return body-frame aerodynamic force [N] and moment [N*m]."""
        coeffs = self.eval_coeffs(
            v_air=v_air,
            alpha_deg=alpha_deg,
            beta_deg=beta_deg,
            delta21_deg=delta21_deg,
            delta22_deg=delta22_deg,
            rho=rho,
            h_m=h_m,
        )

        rho_use = _isa_density(h_m) if rho is None else float(rho)
        qbar = 0.5 * rho_use * float(v_air) ** 2

        f_b = qbar * self.s_ref * np.array(
            [coeffs["cf_x"], coeffs["cf_y"], coeffs["cf_z"]],
            dtype=float,
        )
        f_b = f_b * np.asarray(self.force_sign, dtype=float)
        m_b = qbar * self.s_ref * np.array(
            [
                self.b_ref * coeffs["cm_x"],
                self.c_ref * coeffs["cm_y"],
                self.b_ref * coeffs["cm_z"],
            ],
            dtype=float,
        )
        m_b = m_b * np.asarray(self.moment_sign, dtype=float)

        return f_b, m_b

    def mach_reference(self) -> float | None:
        """Return table Mach reference if present in CSV."""
        return self._mach_ref


if __name__ == "__main__":
    lut = AeroLookupTable.from_csv(
        "aero_lookup.csv",
        s_ref=0.0476,
        b_ref=0.07,
        c_ref=0.07,
        out_of_bounds="clamp",
        force_sign=(-1.0, 1.0, -1.0),
    )

    v_air = 20.0
    alpha_deg = 4.0
    beta_deg = 0.0
    delta21_deg = 0.0
    delta22_deg = 0.0

    coeffs = lut.eval_coeffs(
        v_air=v_air,
        alpha_deg=alpha_deg,
        beta_deg=beta_deg,
        delta21_deg=delta21_deg,
        delta22_deg=delta22_deg,
        h_m=1200.0,
    )
    f_b, m_b = lut.forces_moments(
        v_air=v_air,
        alpha_deg=alpha_deg,
        beta_deg=beta_deg,
        delta21_deg=delta21_deg,
        delta22_deg=delta22_deg,
        h_m=1200.0,
    )

    print("Aero LUT demo")
    print(f"table Mach reference: {lut.mach_reference()}")
    print("coefficients:", coeffs)
    print("F_b [N]:", np.array2string(f_b, precision=3))
    print("M_b [N*m]:", np.array2string(m_b, precision=3))
