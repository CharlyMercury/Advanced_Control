import io
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

HEADERS_3 = "t_s,pwm_cmd,omega_real_rad_s"
HEADERS_4 = "t_s,pwm_cmd,omega_real_rad_s,omega_model_rad_s"


def load_motor_log(path: str) -> pd.DataFrame:
    """
    Reads a text/log file and extracts a valid CSV block.
    Accepts a 3-column or 4-column header.
    """
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        text = f.read()

    lines = text.splitlines()

    start_idx = None
    header = None
    for i, line in enumerate(lines):
        s = line.strip()
        if s == HEADERS_4:
            start_idx = i
            header = HEADERS_4
            break
        if s == HEADERS_3:
            start_idx = i
            header = HEADERS_3
            break

    if start_idx is None:
        raise ValueError(
            "Expected header not found. It must be one of:\n"
            f"  {HEADERS_3}\n"
            f"  {HEADERS_4}"
        )

    ncols = 4 if header == HEADERS_4 else 3
    csv_lines = [header]

    for line in lines[start_idx + 1:]:
        s = line.strip()
        if not s:
            continue
        parts = s.split(",")
        if len(parts) != ncols:
            continue
        try:
            for p in parts:
                float(p)
            csv_lines.append(s)
        except ValueError:
            continue

    if len(csv_lines) <= 1:
        raise ValueError("The header was found, but no valid data rows were found.")

    df = pd.read_csv(io.StringIO("\n".join(csv_lines)))
    return df


def detect_step_window(u: np.ndarray, threshold: float = 0.2):
    """
    Detects a large step in u[k]. Returns (k_on, k_off).
    threshold: minimum jump magnitude to consider an event.
    """
    du = np.diff(u)
    idx = np.where(np.abs(du) > threshold)[0]
    if len(idx) < 1:
        raise ValueError("No step was detected in pwm_cmd. Adjust threshold or inspect the log.")

    k_on = int(idx[0] + 1)
    k_off = int(idx[1] + 1) if len(idx) >= 2 else (len(u) - 1)
    return k_on, k_off


def estimate_first_order_with_offset(t, u, w, k_on, k_off):
    """
    Estimates K, tau, and d for the continuous model:

        dot(w) = -(1/tau) * w + (K/tau) * u + (d/tau)

    using a 63.2% method on a step response.

    Assumption:
      - before the step, u ~= 0 and the motor is close to steady state,
        so the pre-step mean approximates d.
    """
    dt = np.diff(t)
    Ts = float(np.median(dt))

    if k_on < 2 or k_off <= k_on + 4:
        raise ValueError("The detected step window is too short.")

    # Pre-step steady value (used as offset estimate d)
    pre_end = max(1, k_on - 1)
    pre_slice = slice(0, pre_end)
    d = float(np.mean(w[pre_slice]))

    # Step input mean, excluding edges
    u_slice = slice(k_on + 2, max(k_on + 3, k_off - 2))
    u_step = float(np.mean(u[u_slice]))
    if abs(u_step) < 1e-8:
        raise ValueError("u_step is too close to zero. Cannot estimate gain.")

    # Steady-state during the step
    ss_len = max(5, int(1.0 / Ts))
    ss_start = max(k_on + 5, k_off - ss_len)
    ss_slice = slice(ss_start, k_off)
    wss = float(np.mean(w[ss_slice]))

    # Gain around the offset
    K = (wss - d) / u_step

    # 63.2% target from d to wss
    target = d + 0.632 * (wss - d)

    seg_t = t[k_on:k_off]
    seg_w = w[k_on:k_off]

    if (wss - d) >= 0:
        idx_cross = np.where(seg_w >= target)[0]
    else:
        idx_cross = np.where(seg_w <= target)[0]

    if len(idx_cross) == 0:
        tau = 5.0 * Ts
        return K, tau, d, Ts

    i = int(idx_cross[0])
    if i == 0:
        tau = Ts
        return K, tau, d, Ts

    t1, t2 = seg_t[i - 1], seg_t[i]
    w1, w2 = seg_w[i - 1], seg_w[i]

    if abs(w2 - w1) < 1e-12:
        t63 = t2
    else:
        frac = (target - w1) / (w2 - w1)
        t63 = t1 + frac * (t2 - t1)

    t0 = t[k_on]
    tau = float(max(t63 - t0, 1e-4))

    return K, tau, d, Ts


def simulate_first_order_with_offset(t, u, K, tau, d, w_init=0.0):
    """
    Simulates the continuous model with exact ZOH discretization:

        dot(w) = -(1/tau) w + (K/tau) u + (d/tau)

    Discrete form:
        w[k] = alpha * w[k-1] + beta * u[k-1] + gamma

    where:
        alpha = exp(-Ts/tau)
        beta  = K * (1 - alpha)
        gamma = d * (1 - alpha)
    """
    dt = np.diff(t)
    Ts = float(np.median(dt))

    alpha = float(np.exp(-Ts / tau))
    beta = float(K * (1.0 - alpha))
    gamma = float(d * (1.0 - alpha))

    w_model = np.zeros_like(u, dtype=float)
    w_model[0] = float(w_init)

    for k in range(1, len(u)):
        w_model[k] = alpha * w_model[k - 1] + beta * u[k - 1] + gamma

    return w_model, alpha, beta, gamma, Ts


def simulate_discrete_model(t, u, alpha, beta, gamma, w_init=0.0):
    """
    Simulates:
        w[k] = alpha * w[k-1] + beta * u[k-1] + gamma
    """
    w_model = np.zeros_like(u, dtype=float)
    w_model[0] = float(w_init)

    for k in range(1, len(u)):
        w_model[k] = alpha * w_model[k - 1] + beta * u[k - 1] + gamma

    dt = np.diff(t)
    Ts = float(np.median(dt))
    return w_model, Ts


def compute_metrics(y_real, y_model):
    err = y_real - y_model
    rmse = np.sqrt(np.mean(err ** 2))
    mae = np.mean(np.abs(err))

    ss_res = np.sum(err ** 2)
    ss_tot = np.sum((y_real - np.mean(y_real)) ** 2)
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else np.nan

    return {
        "rmse": rmse,
        "mae": mae,
        "r2": r2,
        "max_abs_err": np.max(np.abs(err)),
    }


def plot_compare(t, u, y_real, y_model, title):
    metrics = compute_metrics(y_real, y_model)

    plt.figure(figsize=(10, 5))
    plt.plot(t, y_real, label="Velocidad real")
    plt.plot(t, y_model, label="Velocidad del modelo")
    plt.xlabel("Tiempo [s]")
    plt.ylabel("Velocidad [rad/s]")
    plt.title(title)
    plt.grid(True)
    plt.legend()

    text = (
        f"Muestras: {len(t)}\n"
        f"RMSE: {metrics['rmse']:.4f}\n"
        f"MAE: {metrics['mae']:.4f}\n"
        f"R²: {metrics['r2']:.4f}\n"
        f"Max |error|: {metrics['max_abs_err']:.4f}"
    )
    plt.gca().text(
        0.02, 0.98, text,
        transform=plt.gca().transAxes,
        verticalalignment="top",
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.85),
    )

    plt.figure(figsize=(10, 3))
    plt.plot(t, u, label="PWM aplicado")
    plt.xlabel("Tiempo [s]")
    plt.ylabel("PWM")
    plt.title("Entrada aplicada")
    plt.grid(True)
    plt.legend()

    plt.show()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("archivo_log", help="archivo .txt/.log con CSV del ESP32")
    ap.add_argument("--title", default="Comparación: motor real vs modelo")

    # Manual continuous parameters
    ap.add_argument("--tau", type=float, default=None, help="tau continuo [s]")
    ap.add_argument("--K", type=float, default=None, help="ganancia continua K [rad/s]/u")
    ap.add_argument("--d", type=float, default=None, help="offset continuo d [rad/s]")

    # Manual discrete parameters
    ap.add_argument("--alpha", type=float, default=None, help="alpha discreto")
    ap.add_argument("--beta", type=float, default=None, help="beta discreto")
    ap.add_argument("--gamma", type=float, default=None, help="gamma discreto")

    ap.add_argument("--step_threshold", type=float, default=0.2, help="umbral detección escalón en PWM")
    args = ap.parse_args()

    df = load_motor_log(args.archivo_log)

    t = df["t_s"].to_numpy(dtype=float)
    u = df["pwm_cmd"].to_numpy(dtype=float)
    w_real = df["omega_real_rad_s"].to_numpy(dtype=float)

    # If omega_model_rad_s already exists, just plot it
    if "omega_model_rad_s" in df.columns:
        w_model = df["omega_model_rad_s"].to_numpy(dtype=float)
        plot_compare(t, u, w_real, w_model, args.title)
        return

    dt = np.diff(t)
    Ts = float(np.median(dt))

    # Manual discrete mode: alpha, beta, gamma
    if args.alpha is not None and args.beta is not None and args.gamma is not None:
        alpha = float(args.alpha)
        beta = float(args.beta)
        gamma = float(args.gamma)

        w_model, Ts_sim = simulate_discrete_model(
            t, u, alpha, beta, gamma, w_init=w_real[0]
        )

        print(f"[MANUAL DISCRETE] alpha={alpha:.8f}, beta={beta:.8f}, gamma={gamma:.8f}, Ts≈{Ts_sim:.6f}s")
        plot_compare(t, u, w_real, w_model, args.title)
        return

    # Manual continuous mode: K, tau, d
    if args.K is not None and args.tau is not None and args.d is not None:
        K = float(args.K)
        tau = float(args.tau)
        d = float(args.d)

        w_model, alpha, beta, gamma, Ts_sim = simulate_first_order_with_offset(
            t, u, K, tau, d, w_init=w_real[0]
        )

        print(
            f"[MANUAL CONTINUOUS] K={K:.8f}, tau={tau:.8f}s, d={d:.8f} | "
            f"alpha={alpha:.8f}, beta={beta:.8f}, gamma={gamma:.8f}, Ts≈{Ts_sim:.6f}s"
        )
        plot_compare(t, u, w_real, w_model, args.title)
        return

    # AUTO mode: estimate K, tau, d from a step response
    k_on, k_off = detect_step_window(u, threshold=args.step_threshold)
    K, tau, d, Ts_est = estimate_first_order_with_offset(t, u, w_real, k_on, k_off)

    w_model, alpha, beta, gamma, Ts_sim = simulate_first_order_with_offset(
        t, u, K, tau, d, w_init=w_real[0]
    )

    print("=== Parámetros estimados (AUTO) ===")
    print(f"Step on index : {k_on}  (t≈{t[k_on]:.6f}s)")
    print(f"Step off index: {k_off} (t≈{t[k_off]:.6f}s)")
    print(f"Ts           : {Ts_est:.6f} s")
    print(f"K            : {K:.8f} rad/s/u")
    print(f"tau          : {tau:.8f} s")
    print(f"d            : {d:.8f} rad/s")
    print(f"alpha        : {alpha:.8f}")
    print(f"beta         : {beta:.8f}")
    print(f"gamma        : {gamma:.8f}")

    plot_compare(t, u, w_real, w_model, args.title)


if __name__ == "__main__":
    main()