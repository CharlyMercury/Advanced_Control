#!/usr/bin/env python3
"""
Analyzer for ESP32 PI-state velocity control logs.

Expected CSV header:
t_s,omega_ref_rad_s,omega_meas_rad_s,omega_filt_rad_s,e0_int_error,e1_error,v_ff,v_fb,u_eff,pwm_cmd,delta_cnt,position_cnt
"""

from __future__ import annotations

import argparse
import io
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

EXPECTED_HEADER = (
    "t_s,omega_ref_rad_s,omega_meas_rad_s,omega_filt_rad_s,"
    "e0_int_error,e1_error,v_ff,v_fb,u_eff,pwm_cmd,delta_cnt,position_cnt"
)

EXPECTED_COLUMNS = [
    "t_s",
    "omega_ref_rad_s",
    "omega_meas_rad_s",
    "omega_filt_rad_s",
    "e0_int_error",
    "e1_error",
    "v_ff",
    "v_fb",
    "u_eff",
    "pwm_cmd",
    "delta_cnt",
    "position_cnt",
]

# =========================
# NOMBRES MAS COLOQUIALES
# =========================
PLOT_NAMES = {
    "omega_ref_rad_s": "Referencia",
    "omega_meas_rad_s": "Velocidad medida",
    "omega_filt_rad_s": "Velocidad filtrada",
    "omega_model_rad_s": "Velocidad del modelo",
    "e0_int_error": "Error acumulado",
    "e1_error": "Error instantáneo",
    "tracking_error_filt": "Error de seguimiento",
    "model_error_filt": "Error modelo vs salida",
    "v_ff": "Acción base",
    "v_fb": "Corrección del control",
    "u_eff": "Control efectivo",
    "pwm_cmd": "PWM enviado al motor",
    "delta_cnt": "Cambio de cuentas",
    "position_cnt": "Posición acumulada",
}


def extract_csv_block(text: str) -> str:
    lines = text.splitlines()
    start_idx = None

    for i, line in enumerate(lines):
        if line.strip() == EXPECTED_HEADER:
            start_idx = i
            break

    if start_idx is None:
        raise ValueError(
            "No se encontró el encabezado esperado en el log.\n"
            f"Encabezado esperado:\n{EXPECTED_HEADER}"
        )

    valid_lines = [EXPECTED_HEADER]

    for line in lines[start_idx + 1:]:
        s = line.strip()
        if not s:
            continue

        parts = [p.strip() for p in s.split(",")]
        if len(parts) != len(EXPECTED_COLUMNS):
            continue

        try:
            for value in parts:
                float(value)
        except ValueError:
            continue

        valid_lines.append(",".join(parts))

    if len(valid_lines) <= 1:
        raise ValueError("Se encontró el encabezado, pero no se detectaron filas válidas.")

    return "\n".join(valid_lines)


def load_log(path: Path) -> pd.DataFrame:
    text = path.read_text(encoding="utf-8", errors="ignore")
    csv_text = extract_csv_block(text)
    df = pd.read_csv(io.StringIO(csv_text))

    for col in EXPECTED_COLUMNS:
        df[col] = pd.to_numeric(df[col], errors="coerce")

    df = df.dropna().reset_index(drop=True)

    if df.empty:
        raise ValueError("Después de limpiar, no quedaron datos válidos.")

    # Modelo identificado:
    # omega_dot = -a*omega + b*u_eff + c
    a = 11.11281274
    b = -165.65653960
    c = 0.37449236

    dt_valid = df["t_s"].diff()
    dt_median = float(dt_valid[dt_valid > 0].median()) if (dt_valid > 0).any() else 0.02

    omega_model = np.zeros(len(df), dtype=float)
    omega_model[0] = float(df.loc[0, "omega_filt_rad_s"])

    for k in range(1, len(df)):
        Ts = float(df.loc[k, "t_s"] - df.loc[k - 1, "t_s"])
        if Ts <= 0:
            Ts = dt_median

        omega_prev = float(omega_model[k - 1])
        u_prev = float(df.loc[k - 1, "u_eff"])
        domega = (-a * omega_prev) + (b * u_prev) + c
        omega_model[k] = omega_prev + Ts * domega

    df["omega_model_rad_s"] = omega_model
    df["tracking_error_meas"] = df["omega_ref_rad_s"] - df["omega_meas_rad_s"]
    df["tracking_error_filt"] = df["omega_ref_rad_s"] - df["omega_filt_rad_s"]
    df["model_error_filt"] = df["omega_filt_rad_s"] - df["omega_model_rad_s"]

    return df


def compute_metrics(df: pd.DataFrame) -> dict:
    ref = df["omega_ref_rad_s"].to_numpy()
    meas = df["omega_meas_rad_s"].to_numpy()
    filt = df["omega_filt_rad_s"].to_numpy()
    model = df["omega_model_rad_s"].to_numpy()

    return {
        "samples": int(len(df)),
        "t_total_s": float(df["t_s"].iloc[-1] - df["t_s"].iloc[0]),
        "mae_ref_meas": float(np.mean(np.abs(ref - meas))),
        "rmse_ref_meas": float(np.sqrt(np.mean((ref - meas) ** 2))),
        "mae_ref_filt": float(np.mean(np.abs(ref - filt))),
        "rmse_ref_filt": float(np.sqrt(np.mean((ref - filt) ** 2))),
        "mae_filt_model": float(np.mean(np.abs(filt - model))),
        "rmse_filt_model": float(np.sqrt(np.mean((filt - model) ** 2))),
        "max_abs_pwm_cmd": float(np.max(np.abs(df["pwm_cmd"]))),
        "max_abs_u_eff": float(np.max(np.abs(df["u_eff"]))),
        "max_abs_e0": float(np.max(np.abs(df["e0_int_error"]))),
        "max_abs_e1": float(np.max(np.abs(df["e1_error"]))),
    }


def save_speed_plot(df: pd.DataFrame, path: Path) -> None:
    plt.figure(figsize=(12, 6))
    plt.plot(df["t_s"], df["omega_ref_rad_s"], label=PLOT_NAMES["omega_ref_rad_s"])
    plt.plot(df["t_s"], df["omega_meas_rad_s"], label=PLOT_NAMES["omega_meas_rad_s"])
    plt.plot(df["t_s"], df["omega_filt_rad_s"], label=PLOT_NAMES["omega_filt_rad_s"])
    plt.plot(df["t_s"], df["omega_model_rad_s"], label=PLOT_NAMES["omega_model_rad_s"])
    plt.xlabel("Tiempo [s]")
    plt.ylabel("Velocidad angular [rad/s]")
    plt.title("Comparación de velocidades")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=160)
    plt.close()


def save_error_plot(df: pd.DataFrame, path: Path) -> None:
    plt.figure(figsize=(12, 6))
    plt.plot(df["t_s"], df["e1_error"], label=PLOT_NAMES["e1_error"])
    plt.plot(df["t_s"], df["e0_int_error"], label=PLOT_NAMES["e0_int_error"])
    plt.plot(df["t_s"], df["tracking_error_filt"], label=PLOT_NAMES["tracking_error_filt"])
    plt.plot(df["t_s"], df["model_error_filt"], label=PLOT_NAMES["model_error_filt"])
    plt.xlabel("Tiempo [s]")
    plt.ylabel("Error")
    plt.title("Comportamiento del error")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=160)
    plt.close()


def save_control_plot(df: pd.DataFrame, path: Path) -> None:
    plt.figure(figsize=(12, 6))
    plt.plot(df["t_s"], df["v_ff"], label=PLOT_NAMES["v_ff"])
    plt.plot(df["t_s"], df["v_fb"], label=PLOT_NAMES["v_fb"])
    plt.plot(df["t_s"], df["u_eff"], label=PLOT_NAMES["u_eff"])
    plt.plot(df["t_s"], df["pwm_cmd"], label=PLOT_NAMES["pwm_cmd"])
    plt.xlabel("Tiempo [s]")
    plt.ylabel("Señal")
    plt.title("Señales de control")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=160)
    plt.close()


def save_counts_plot(df: pd.DataFrame, path: Path) -> None:
    plt.figure(figsize=(12, 6))
    plt.plot(df["t_s"], df["delta_cnt"], label=PLOT_NAMES["delta_cnt"])
    plt.plot(df["t_s"], df["position_cnt"], label=PLOT_NAMES["position_cnt"])
    plt.xlabel("Tiempo [s]")
    plt.ylabel("Cuentas")
    plt.title("Lectura del encoder")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=160)
    plt.close()


def save_summary_plot(df: pd.DataFrame, path: Path) -> None:
    fig, axes = plt.subplots(4, 1, figsize=(12, 14), sharex=True)

    axes[0].plot(df["t_s"], df["omega_ref_rad_s"], label=PLOT_NAMES["omega_ref_rad_s"])
    axes[0].plot(df["t_s"], df["omega_filt_rad_s"], label=PLOT_NAMES["omega_filt_rad_s"])
    axes[0].plot(df["t_s"], df["omega_model_rad_s"], label=PLOT_NAMES["omega_model_rad_s"])
    axes[0].set_ylabel("Velocidad")
    axes[0].set_title("Resumen general")
    axes[0].grid(True)
    axes[0].legend()

    axes[1].plot(df["t_s"], df["e1_error"], label=PLOT_NAMES["e1_error"])
    axes[1].plot(df["t_s"], df["e0_int_error"], label=PLOT_NAMES["e0_int_error"])
    axes[1].set_ylabel("Error")
    axes[1].grid(True)
    axes[1].legend()

    axes[2].plot(df["t_s"], df["v_ff"], label=PLOT_NAMES["v_ff"])
    axes[2].plot(df["t_s"], df["v_fb"], label=PLOT_NAMES["v_fb"])
    axes[2].plot(df["t_s"], df["u_eff"], label=PLOT_NAMES["u_eff"])
    axes[2].set_ylabel("Control")
    axes[2].grid(True)
    axes[2].legend()

    axes[3].plot(df["t_s"], df["pwm_cmd"], label=PLOT_NAMES["pwm_cmd"])
    axes[3].plot(df["t_s"], df["delta_cnt"], label=PLOT_NAMES["delta_cnt"])
    axes[3].set_xlabel("Tiempo [s]")
    axes[3].set_ylabel("PWM / cuentas")
    axes[3].grid(True)
    axes[3].legend()

    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def save_all_in_one_plot(df: pd.DataFrame, path: Path) -> None:
    plt.figure(figsize=(16, 8))
    plt.plot(df["t_s"], df["omega_ref_rad_s"], label=PLOT_NAMES["omega_ref_rad_s"])
    plt.plot(df["t_s"], df["omega_meas_rad_s"], label=PLOT_NAMES["omega_meas_rad_s"])
    plt.plot(df["t_s"], df["omega_filt_rad_s"], label=PLOT_NAMES["omega_filt_rad_s"])
    plt.plot(df["t_s"], df["omega_model_rad_s"], label=PLOT_NAMES["omega_model_rad_s"])
    plt.plot(df["t_s"], df["e0_int_error"], label=PLOT_NAMES["e0_int_error"])
    plt.plot(df["t_s"], df["e1_error"], label=PLOT_NAMES["e1_error"])
    plt.plot(df["t_s"], df["v_ff"], label=PLOT_NAMES["v_ff"])
    plt.plot(df["t_s"], df["v_fb"], label=PLOT_NAMES["v_fb"])
    plt.plot(df["t_s"], df["u_eff"], label=PLOT_NAMES["u_eff"])
    plt.plot(df["t_s"], df["pwm_cmd"], label=PLOT_NAMES["pwm_cmd"])
    plt.xlabel("Tiempo [s]")
    plt.ylabel("Valor de la señal")
    plt.title("Todas las señales en una sola gráfica")
    plt.grid(True)
    plt.legend(loc="best", ncol=2)
    plt.tight_layout()
    plt.savefig(path, dpi=160)
    plt.close()


def save_ref_vs_speed_plot(df: pd.DataFrame, path: Path) -> None:
    plt.figure(figsize=(14, 6))
    plt.plot(df["t_s"], df["omega_ref_rad_s"], label=PLOT_NAMES["omega_ref_rad_s"])
    plt.plot(df["t_s"], df["omega_filt_rad_s"], label=PLOT_NAMES["omega_filt_rad_s"])
    plt.xlabel("Tiempo [s]")
    plt.ylabel("Velocidad angular [rad/s]")
    plt.title("Referencia contra velocidad de salida")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=160)
    plt.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze PI-state velocity control logs.")
    parser.add_argument("logfile", help="Path to serial log file")
    parser.add_argument("--prefix", default="pi_state_analysis", help="Output prefix")
    parser.add_argument("--show", action="store_true", help="Show plots interactively")
    args = parser.parse_args()

    log_path = Path(args.logfile)
    if not log_path.exists():
        raise FileNotFoundError(f"No existe el archivo: {log_path}")

    df = load_log(log_path)
    metrics = compute_metrics(df)

    prefix = Path(args.prefix)
    enriched_csv = Path(f"{prefix}_enriched.csv")
    speed_png = Path(f"{prefix}_speed.png")
    errors_png = Path(f"{prefix}_errors.png")
    control_png = Path(f"{prefix}_control.png")
    counts_png = Path(f"{prefix}_counts.png")
    summary_png = Path(f"{prefix}_summary.png")
    all_in_one_png = Path(f"{prefix}_all_in_one.png")
    ref_vs_speed_png = Path(f"{prefix}_ref_vs_speed.png")

    df.to_csv(enriched_csv, index=False)

    save_speed_plot(df, speed_png)
    save_error_plot(df, errors_png)
    save_control_plot(df, control_png)
    save_counts_plot(df, counts_png)
    save_summary_plot(df, summary_png)
    save_all_in_one_plot(df, all_in_one_png)
    save_ref_vs_speed_plot(df, ref_vs_speed_png)

    print("=== Metrics ===")
    for key, value in metrics.items():
        if isinstance(value, float):
            print(f"{key}: {value:.6f}")
        else:
            print(f"{key}: {value}")

    print("\nGenerated files:")
    print(enriched_csv)
    print(speed_png)
    print(errors_png)
    print(control_png)
    print(counts_png)
    print(summary_png)
    print(all_in_one_png)
    print(ref_vs_speed_png)

    if args.show:
        plt.figure(figsize=(12, 6))
        plt.plot(df["t_s"], df["omega_ref_rad_s"], label=PLOT_NAMES["omega_ref_rad_s"])
        plt.plot(df["t_s"], df["omega_meas_rad_s"], label=PLOT_NAMES["omega_meas_rad_s"])
        plt.plot(df["t_s"], df["omega_filt_rad_s"], label=PLOT_NAMES["omega_filt_rad_s"])
        plt.plot(df["t_s"], df["omega_model_rad_s"], label=PLOT_NAMES["omega_model_rad_s"])
        plt.xlabel("Tiempo [s]")
        plt.ylabel("Velocidad angular [rad/s]")
        plt.title("Comparación de velocidades")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()

        plt.figure(figsize=(12, 6))
        plt.plot(df["t_s"], df["e1_error"], label=PLOT_NAMES["e1_error"])
        plt.plot(df["t_s"], df["e0_int_error"], label=PLOT_NAMES["e0_int_error"])
        plt.plot(df["t_s"], df["tracking_error_filt"], label=PLOT_NAMES["tracking_error_filt"])
        plt.plot(df["t_s"], df["model_error_filt"], label=PLOT_NAMES["model_error_filt"])
        plt.xlabel("Tiempo [s]")
        plt.ylabel("Error")
        plt.title("Comportamiento del error")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()

        plt.figure(figsize=(12, 6))
        plt.plot(df["t_s"], df["v_ff"], label=PLOT_NAMES["v_ff"])
        plt.plot(df["t_s"], df["v_fb"], label=PLOT_NAMES["v_fb"])
        plt.plot(df["t_s"], df["u_eff"], label=PLOT_NAMES["u_eff"])
        plt.plot(df["t_s"], df["pwm_cmd"], label=PLOT_NAMES["pwm_cmd"])
        plt.xlabel("Tiempo [s]")
        plt.ylabel("Señal")
        plt.title("Señales de control")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()

        plt.figure(figsize=(16, 8))
        plt.plot(df["t_s"], df["omega_ref_rad_s"], label=PLOT_NAMES["omega_ref_rad_s"])
        plt.plot(df["t_s"], df["omega_meas_rad_s"], label=PLOT_NAMES["omega_meas_rad_s"])
        plt.plot(df["t_s"], df["omega_filt_rad_s"], label=PLOT_NAMES["omega_filt_rad_s"])
        plt.plot(df["t_s"], df["omega_model_rad_s"], label=PLOT_NAMES["omega_model_rad_s"])
        plt.plot(df["t_s"], df["e0_int_error"], label=PLOT_NAMES["e0_int_error"])
        plt.plot(df["t_s"], df["e1_error"], label=PLOT_NAMES["e1_error"])
        plt.plot(df["t_s"], df["v_ff"], label=PLOT_NAMES["v_ff"])
        plt.plot(df["t_s"], df["v_fb"], label=PLOT_NAMES["v_fb"])
        plt.plot(df["t_s"], df["u_eff"], label=PLOT_NAMES["u_eff"])
        plt.plot(df["t_s"], df["pwm_cmd"], label=PLOT_NAMES["pwm_cmd"])
        plt.xlabel("Tiempo [s]")
        plt.ylabel("Valor de la señal")
        plt.title("Todas las señales en una sola gráfica")
        plt.grid(True)
        plt.legend(loc="best", ncol=2)
        plt.tight_layout()

        plt.figure(figsize=(14, 6))
        plt.plot(df["t_s"], df["omega_ref_rad_s"], label=PLOT_NAMES["omega_ref_rad_s"])
        plt.plot(df["t_s"], df["omega_filt_rad_s"], label=PLOT_NAMES["omega_filt_rad_s"])
        plt.xlabel("Tiempo [s]")
        plt.ylabel("Velocidad angular [rad/s]")
        plt.title("Referencia contra velocidad de salida")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()

        plt.show()


if __name__ == "__main__":
    main()