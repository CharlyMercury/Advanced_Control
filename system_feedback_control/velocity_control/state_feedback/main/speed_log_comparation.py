#!/usr/bin/env python3
"""Analiza un log serial/printf del control de velocidad y genera comparativas con Matplotlib.

Encabezado esperado:
    t_s,omega_ref_rad_s,omega_meas_rad_s,omega_filt_rad_s,error_rad_s,u_eff,pwm_cmd,delta_cnt,position_cnt

Qué hace:
- Extrae el bloque CSV de un log con ruido.
- Simula el modelo identificado usando u_eff.
- Genera comparativas con Matplotlib.
- Guarda un CSV enriquecido con omega_model y errores.
"""

from __future__ import annotations

import argparse
import io
import math
import re
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

EXPECTED_HEADER = (
    "t_s,omega_ref_rad_s,omega_meas_rad_s,"
    "omega_filt_rad_s,error_rad_s,u_eff,pwm_cmd,delta_cnt,position_cnt"
)

# Modelo identificado:
# omega_dot = -a*omega + b*u_eff + c
MODEL_A = 11.11281274
MODEL_B = -165.65653960
MODEL_C = 0.37449236


def extract_csv_block(path: Path) -> pd.DataFrame:
    text = path.read_text(encoding="utf-8", errors="ignore")
    lines = text.splitlines()

    start_idx = None
    for i, raw in enumerate(lines):
        if raw.strip() == EXPECTED_HEADER:
            start_idx = i
            break

    if start_idx is None:
        raise ValueError(
            "No se encontró el encabezado esperado en el log.\n"
            f"Encabezado esperado:\n{EXPECTED_HEADER}"
        )

    ncols = len(EXPECTED_HEADER.split(","))
    numeric_row = re.compile(r"^[\s+\-0-9eE.,]+$")
    csv_lines = [EXPECTED_HEADER]

    for raw in lines[start_idx + 1 :]:
        s = raw.strip()
        if not s:
            continue
        if not numeric_row.match(s):
            continue
        parts = [p.strip() for p in s.split(",")]
        if len(parts) != ncols:
            continue
        try:
            [float(p) for p in parts]
        except ValueError:
            continue
        csv_lines.append(",".join(parts))

    if len(csv_lines) < 3:
        raise ValueError("Se encontró el encabezado, pero no hay suficientes datos válidos.")

    return pd.read_csv(io.StringIO("\n".join(csv_lines)))


def validate_and_prepare(df: pd.DataFrame) -> pd.DataFrame:
    required = EXPECTED_HEADER.split(",")
    missing = [c for c in required if c not in df.columns]
    if missing:
        raise ValueError(f"Faltan columnas en el log: {missing}")

    df = df.copy().dropna().sort_values("t_s", kind="stable").reset_index(drop=True)
    df = df.loc[~df["t_s"].duplicated()].reset_index(drop=True)

    if len(df) < 3:
        raise ValueError("No hay suficientes muestras tras limpiar el log.")

    return df


def simulate_model(df: pd.DataFrame, a: float, b: float, c: float) -> np.ndarray:
    t = df["t_s"].to_numpy(dtype=float)
    u = df["u_eff"].to_numpy(dtype=float)
    omega_meas = df["omega_meas_rad_s"].to_numpy(dtype=float)

    omega_model = np.zeros_like(omega_meas)
    omega_model[0] = omega_meas[0]

    if len(t) > 1:
        dt_default = float(np.median(np.diff(t)))
    else:
        dt_default = 0.02

    for k in range(len(df) - 1):
        dt = t[k + 1] - t[k]
        if not np.isfinite(dt) or dt <= 0.0:
            dt = dt_default
        domega = (-a * omega_model[k]) + (b * u[k]) + c
        omega_model[k + 1] = omega_model[k] + dt * domega

    return omega_model


def compute_metrics(df: pd.DataFrame, omega_model: np.ndarray) -> dict[str, float]:
    ref = df["omega_ref_rad_s"].to_numpy(dtype=float)
    meas = df["omega_meas_rad_s"].to_numpy(dtype=float)
    filt = df["omega_filt_rad_s"].to_numpy(dtype=float)

    err_ref_meas = ref - meas
    err_ref_filt = ref - filt
    err_model_meas = omega_model - meas

    return {
        "samples": float(len(df)),
        "duration_s": float(df["t_s"].iloc[-1] - df["t_s"].iloc[0]),
        "mae_ref_meas": float(np.mean(np.abs(err_ref_meas))),
        "rmse_ref_meas": float(np.sqrt(np.mean(err_ref_meas**2))),
        "mae_ref_filt": float(np.mean(np.abs(err_ref_filt))),
        "rmse_ref_filt": float(np.sqrt(np.mean(err_ref_filt**2))),
        "mae_model_meas": float(np.mean(np.abs(err_model_meas))),
        "rmse_model_meas": float(np.sqrt(np.mean(err_model_meas**2))),
        "max_abs_error_ref_meas": float(np.max(np.abs(err_ref_meas))),
        "steady_state_error_last10pct": float(
            np.mean(np.abs(err_ref_meas[max(0, int(0.9 * len(df))) :]))
        ),
        "mean_abs_pwm": float(np.mean(np.abs(df["pwm_cmd"].to_numpy(dtype=float)))),
        "mean_abs_u_eff": float(np.mean(np.abs(df["u_eff"].to_numpy(dtype=float)))),
    }


def estimate_step_metrics(df: pd.DataFrame) -> dict[str, float]:
    t = df["t_s"].to_numpy(dtype=float)
    ref = df["omega_ref_rad_s"].to_numpy(dtype=float)
    meas = df["omega_meas_rad_s"].to_numpy(dtype=float)

    idx_candidates = np.where(np.abs(np.diff(ref)) > 1e-9)[0]
    if len(idx_candidates) == 0:
        return {}

    k0 = int(idx_candidates[0] + 1)
    ref0 = ref[max(k0 - 1, 0)]
    ref1 = ref[k0]
    amp = ref1 - ref0
    if abs(amp) < 1e-9:
        return {}

    target10 = ref0 + 0.10 * amp
    target90 = ref0 + 0.90 * amp

    def first_crossing(y: np.ndarray, target: float, start: int) -> int | None:
        for i in range(start, len(y)):
            if (amp >= 0 and y[i] >= target) or (amp < 0 and y[i] <= target):
                return i
        return None

    i10 = first_crossing(meas, target10, k0)
    i90 = first_crossing(meas, target90, k0)

    peak = float(np.max(meas[k0:])) if amp >= 0 else float(np.min(meas[k0:]))
    overshoot = max(0.0, abs((peak - ref1) / amp) * 100.0) if abs(amp) > 1e-9 else 0.0

    settling_time = math.nan
    band = 0.02 * abs(amp)
    if band > 0.0:
        for i in range(k0, len(meas)):
            if np.all(np.abs(meas[i:] - ref1) <= band):
                settling_time = float(t[i] - t[k0])
                break

    out = {
        "step_time_s": float(t[k0]),
        "step_amplitude": float(amp),
        "overshoot_pct": float(overshoot),
    }
    if i10 is not None and i90 is not None:
        out["rise_time_10_90_s"] = float(t[i90] - t[i10])
    if np.isfinite(settling_time):
        out["settling_time_2pct_s"] = float(settling_time)
    return out


def save_augmented_csv(df: pd.DataFrame, omega_model: np.ndarray, output_path: Path) -> None:
    out = df.copy()
    out["omega_model_rad_s"] = omega_model
    out["error_ref_meas_rad_s"] = out["omega_ref_rad_s"] - out["omega_meas_rad_s"]
    out["error_ref_filt_rad_s"] = out["omega_ref_rad_s"] - out["omega_filt_rad_s"]
    out["error_model_meas_rad_s"] = out["omega_model_rad_s"] - out["omega_meas_rad_s"]
    out.to_csv(output_path, index=False)


def plot_speed_comparison(df: pd.DataFrame, omega_model: np.ndarray, output_path: Path) -> None:
    t = df["t_s"].to_numpy(dtype=float)

    plt.figure(figsize=(12, 6))
    plt.plot(t, df["omega_ref_rad_s"], label="omega_ref")
    plt.plot(t, df["omega_meas_rad_s"], label="omega_meas")
    plt.plot(t, df["omega_filt_rad_s"], label="omega_filt")
    plt.plot(t, omega_model, label="omega_model")
    plt.title("Comparativa de velocidades")
    plt.xlabel("Tiempo [s]")
    plt.ylabel("Velocidad [rad/s]")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close()


def plot_error_comparison(df: pd.DataFrame, omega_model: np.ndarray, output_path: Path) -> None:
    t = df["t_s"].to_numpy(dtype=float)

    plt.figure(figsize=(12, 6))
    plt.plot(t, df["error_rad_s"], label="error_logged")
    plt.plot(t, df["omega_ref_rad_s"] - df["omega_meas_rad_s"], label="ref - meas")
    plt.plot(t, df["omega_ref_rad_s"] - df["omega_filt_rad_s"], label="ref - filt")
    plt.plot(t, omega_model - df["omega_meas_rad_s"], label="model - meas")
    plt.title("Comparativa de errores")
    plt.xlabel("Tiempo [s]")
    plt.ylabel("Error [rad/s]")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close()


def plot_control_comparison(df: pd.DataFrame, output_path: Path) -> None:
    t = df["t_s"].to_numpy(dtype=float)

    plt.figure(figsize=(12, 6))
    plt.plot(t, df["u_eff"], label="u_eff")
    plt.plot(t, df["pwm_cmd"], label="pwm_cmd")
    plt.title("Comparativa de señales de control")
    plt.xlabel("Tiempo [s]")
    plt.ylabel("Comando")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close()


def plot_all_in_one(df: pd.DataFrame, omega_model: np.ndarray, output_path: Path) -> None:
    t = df["t_s"].to_numpy(dtype=float)

    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    axes[0].plot(t, df["omega_ref_rad_s"], label="omega_ref")
    axes[0].plot(t, df["omega_meas_rad_s"], label="omega_meas")
    axes[0].plot(t, df["omega_filt_rad_s"], label="omega_filt")
    axes[0].plot(t, omega_model, label="omega_model")
    axes[0].set_title("Comparativa de velocidades")
    axes[0].set_ylabel("Velocidad [rad/s]")
    axes[0].grid(True)
    axes[0].legend()

    axes[1].plot(t, df["error_rad_s"], label="error_logged")
    axes[1].plot(t, df["omega_ref_rad_s"] - df["omega_meas_rad_s"], label="ref - meas")
    axes[1].plot(t, df["omega_ref_rad_s"] - df["omega_filt_rad_s"], label="ref - filt")
    axes[1].plot(t, omega_model - df["omega_meas_rad_s"], label="model - meas")
    axes[1].set_title("Comparativa de errores")
    axes[1].set_ylabel("Error [rad/s]")
    axes[1].grid(True)
    axes[1].legend()

    axes[2].plot(t, df["u_eff"], label="u_eff")
    axes[2].plot(t, df["pwm_cmd"], label="pwm_cmd")
    axes[2].set_title("Comparativa de señales de control")
    axes[2].set_xlabel("Tiempo [s]")
    axes[2].set_ylabel("Comando")
    axes[2].grid(True)
    axes[2].legend()

    fig.tight_layout()
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def print_metrics(metrics: dict[str, float], step_metrics: dict[str, float]) -> None:
    print("\n=== Resumen de métricas ===")
    for key, value in metrics.items():
        if key == "samples":
            print(f"{key}: {int(value)}")
        else:
            print(f"{key}: {value:.6f}")

    if step_metrics:
        print("\n=== Métricas de escalón ===")
        for key, value in step_metrics.items():
            print(f"{key}: {value:.6f}")


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Toma un log serial/printf del control de velocidad y genera comparativas con Matplotlib."
        )
    )
    parser.add_argument("input", type=Path, help="Ruta al log (.txt, .log, etc.)")
    parser.add_argument("--prefix", type=Path, default=Path("comparison"), help="Prefijo para archivos de salida")
    parser.add_argument("--a", type=float, default=MODEL_A, help="Parámetro a del modelo")
    parser.add_argument("--b", type=float, default=MODEL_B, help="Parámetro b del modelo")
    parser.add_argument("--c", type=float, default=MODEL_C, help="Parámetro c del modelo")
    parser.add_argument("--show", action="store_true", help="Muestra la figura resumen al final")
    return parser


def main() -> None:
    parser = build_argparser()
    args = parser.parse_args()

    df = extract_csv_block(args.input)
    df = validate_and_prepare(df)
    omega_model = simulate_model(df, args.a, args.b, args.c)

    metrics = compute_metrics(df, omega_model)
    step_metrics = estimate_step_metrics(df)

    csv_path = args.prefix.with_name(args.prefix.name + "_enriched.csv")
    speed_png = args.prefix.with_name(args.prefix.name + "_speed.png")
    error_png = args.prefix.with_name(args.prefix.name + "_errors.png")
    control_png = args.prefix.with_name(args.prefix.name + "_control.png")
    summary_png = args.prefix.with_name(args.prefix.name + "_summary.png")

    save_augmented_csv(df, omega_model, csv_path)
    plot_speed_comparison(df, omega_model, speed_png)
    plot_error_comparison(df, omega_model, error_png)
    plot_control_comparison(df, control_png)
    plot_all_in_one(df, omega_model, summary_png)

    print_metrics(metrics, step_metrics)

    print("\nArchivos generados:")
    print(f"- CSV enriquecido: {csv_path}")
    print(f"- Comparativa velocidad: {speed_png}")
    print(f"- Comparativa errores:   {error_png}")
    print(f"- Comparativa control:   {control_png}")
    print(f"- Resumen completo:      {summary_png}")

    if args.show:
        t = df["t_s"].to_numpy(dtype=float)
        fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

        axes[0].plot(t, df["omega_ref_rad_s"], label="omega_ref")
        axes[0].plot(t, df["omega_meas_rad_s"], label="omega_meas")
        axes[0].plot(t, df["omega_filt_rad_s"], label="omega_filt")
        axes[0].plot(t, omega_model, label="omega_model")
        axes[0].set_title("Comparativa de velocidades")
        axes[0].set_ylabel("Velocidad [rad/s]")
        axes[0].grid(True)
        axes[0].legend()

        axes[1].plot(t, df["error_rad_s"], label="error_logged")
        axes[1].plot(t, df["omega_ref_rad_s"] - df["omega_meas_rad_s"], label="ref - meas")
        axes[1].plot(t, df["omega_ref_rad_s"] - df["omega_filt_rad_s"], label="ref - filt")
        axes[1].plot(t, omega_model - df["omega_meas_rad_s"], label="model - meas")
        axes[1].set_title("Comparativa de errores")
        axes[1].set_ylabel("Error [rad/s]")
        axes[1].grid(True)
        axes[1].legend()

        axes[2].plot(t, df["u_eff"], label="u_eff")
        axes[2].plot(t, df["pwm_cmd"], label="pwm_cmd")
        axes[2].set_title("Comparativa de señales de control")
        axes[2].set_xlabel("Tiempo [s]")
        axes[2].set_ylabel("Comando")
        axes[2].grid(True)
        axes[2].legend()

        fig.tight_layout()
        plt.show()


if __name__ == "__main__":
    main()
