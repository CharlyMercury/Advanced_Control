import io
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

HEADER = "k,t_k1_s,u_eff_k,omega_k_rad_s,omega_k1_rad_s"

A_EST = 11.25657071
B_EST = 167.16911397
C_EST = -0.42476213


def load_validation_log(path: str) -> pd.DataFrame:
    with open(path, "r", encoding="utf-8-sig", errors="ignore") as f:
        text = f.read()

    lines = text.splitlines()

    start_idx = None
    for i, line in enumerate(lines):
        s = line.strip().replace("\ufeff", "")
        if s == HEADER:
            start_idx = i
            break

    if start_idx is None:
        raise ValueError(
            "Expected header not found. It must be:\n"
            f"  {HEADER}"
        )

    csv_lines = [HEADER]
    expected_cols = 5

    for line in lines[start_idx + 1:]:
        s = line.strip().replace("\ufeff", "")
        if not s:
            continue

        parts = [p.strip() for p in s.split(",")]
        if len(parts) != expected_cols:
            continue

        try:
            [float(x) for x in parts]
            csv_lines.append(",".join(parts))
        except ValueError:
            continue

    if len(csv_lines) <= 1:
        raise ValueError("The header was found, but no valid data rows were found.")

    df = pd.read_csv(io.StringIO("\n".join(csv_lines)))

    required = ["k", "t_k1_s", "u_eff_k", "omega_k_rad_s", "omega_k1_rad_s"]
    missing = [col for col in required if col not in df.columns]
    if missing:
        raise ValueError(f"Missing required columns: {missing}")

    return df


def continuous_to_discrete(a: float, b: float, c: float, Ts: float):
    if a <= 0.0:
        raise ValueError("Parameter 'a' must be > 0.")

    alpha = float(np.exp(-a * Ts))
    beta = float((b / a) * (1.0 - alpha))
    gamma = float((c / a) * (1.0 - alpha))
    return alpha, beta, gamma


def simulate_model(
    u_eff_k: np.ndarray,
    omega_k0: float,
    alpha: float,
    beta: float,
    gamma: float,
) -> np.ndarray:
    omega_model_k1 = np.zeros_like(u_eff_k, dtype=float)
    omega = float(omega_k0)

    for k in range(len(u_eff_k)):
        omega = alpha * omega + beta * u_eff_k[k] + gamma
        omega_model_k1[k] = omega

    return omega_model_k1


def compute_metrics(y_real: np.ndarray, y_model: np.ndarray) -> dict:
    err = y_real - y_model
    rmse = np.sqrt(np.mean(err ** 2))
    mae = np.mean(np.abs(err))

    ss_res = np.sum(err ** 2)
    ss_tot = np.sum((y_real - np.mean(y_real)) ** 2)
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0.0 else np.nan

    return {
        "rmse": float(rmse),
        "mae": float(mae),
        "r2": float(r2),
        "max_abs_err": float(np.max(np.abs(err))),
    }


def plot_compare(t_k1, u_eff_k, omega_real_k1, omega_model_k1, title):
    metrics = compute_metrics(omega_real_k1, omega_model_k1)

    fig, axes = plt.subplots(
        2,
        1,
        figsize=(12, 8),
        sharex=True,
        gridspec_kw={"height_ratios": [1, 3]},
    )

    axes[0].plot(t_k1, u_eff_k, label="u_eff[k]")
    axes[0].set_ylabel("u_eff")
    axes[0].set_title("Entrada efectiva del experimento")
    axes[0].grid(True)
    axes[0].legend()

    axes[1].plot(t_k1, omega_real_k1, label="Velocidad real")
    axes[1].plot(t_k1, omega_model_k1, label="Velocidad del modelo")
    axes[1].set_xlabel("Tiempo [s]")
    axes[1].set_ylabel("Velocidad [rad/s]")
    axes[1].set_title(title)
    axes[1].grid(True)
    axes[1].legend()

    text = (
        f"Muestras: {len(t_k1)}\n"
        f"RMSE: {metrics['rmse']:.6f}\n"
        f"MAE: {metrics['mae']:.6f}\n"
        f"R²: {metrics['r2']:.6f}\n"
        f"Max |error|: {metrics['max_abs_err']:.6f}"
    )
    axes[1].text(
        0.02,
        0.98,
        text,
        transform=axes[1].transAxes,
        verticalalignment="top",
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.85),
    )

    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("archivo_log", help="archivo .txt/.log con CSV del experimento")
    parser.add_argument(
        "--title",
        default="Comparación en u_eff: velocidad real vs simulación del modelo",
    )
    parser.add_argument("--a", type=float, default=A_EST)
    parser.add_argument("--b", type=float, default=B_EST)
    parser.add_argument("--c", type=float, default=C_EST)
    args = parser.parse_args()

    df = load_validation_log(args.archivo_log)

    t_k1 = df["t_k1_s"].to_numpy(dtype=float)
    u_eff_k = df["u_eff_k"].to_numpy(dtype=float)
    omega_k = df["omega_k_rad_s"].to_numpy(dtype=float)
    omega_real_k1 = df["omega_k1_rad_s"].to_numpy(dtype=float)

    dt = np.diff(t_k1)
    Ts = float(np.median(dt)) if len(dt) > 0 else 0.02

    alpha, beta, gamma = continuous_to_discrete(args.a, args.b, args.c, Ts)
    omega_model_k1 = simulate_model(u_eff_k, omega_k[0], alpha, beta, gamma)

    print("=== Comparación del modelo trabajando en u_eff ===")
    print()

    print("=== Parámetros continuos ===")
    print(f"a = {args.a:.8f}")
    print(f"b = {args.b:.8f}")
    print(f"c = {args.c:.8f}")
    print()

    print("=== Parámetros discretos ===")
    print(f"Ts    = {Ts:.8f} s")
    print(f"alpha = {alpha:.8f}")
    print(f"beta  = {beta:.8f}")
    print(f"gamma = {gamma:.8f}")
    print()

    print("=== Rango de entrada efectiva ===")
    print(f"u_eff min = {np.min(u_eff_k):.6f}")
    print(f"u_eff max = {np.max(u_eff_k):.6f}")
    print()

    metrics = compute_metrics(omega_real_k1, omega_model_k1)

    print("=== Métricas ===")
    print(f"RMSE      = {metrics['rmse']:.8f}")
    print(f"MAE       = {metrics['mae']:.8f}")
    print(f"R^2       = {metrics['r2']:.8f}")
    print(f"Max error = {metrics['max_abs_err']:.8f}")

    plot_compare(
        t_k1=t_k1,
        u_eff_k=u_eff_k,
        omega_real_k1=omega_real_k1,
        omega_model_k1=omega_model_k1,
        title=args.title,
    )


if __name__ == "__main__":
    main()
