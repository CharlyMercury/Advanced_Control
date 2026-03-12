import io
import sys
import numpy as np
import pandas as pd


DEFAULT_TS = 0.02  # 20 ms


def load_dataset(path: str) -> pd.DataFrame:
    """
    Reads a serial log file and extracts only the CSV table.

    Supported headers:
        phi0_omega_k,phi1_u_k,phi2_bias,y_omega_k1
        phi0_omega_k,phi1_u_k,y_omega_k1   (legacy format)

    Any garbage text before/after the CSV is ignored.
    """
    text = open(path, "r", encoding="utf-8", errors="ignore").read()
    lines = text.splitlines()

    new_header = "phi0_omega_k,phi1_u_k,phi2_bias,y_omega_k1"
    old_header = "phi0_omega_k,phi1_u_k,y_omega_k1"

    start_idx = None
    detected_header = None

    for i, line in enumerate(lines):
        s = line.strip()
        if s == new_header:
            start_idx = i
            detected_header = new_header
            break
        if s == old_header:
            start_idx = i
            detected_header = old_header
            break

    if start_idx is None:
        raise ValueError(
            "Expected CSV header not found. Supported headers are:\n"
            f"  '{new_header}'\n"
            f"  '{old_header}'"
        )

    csv_lines = [detected_header]

    expected_cols = 4 if detected_header == new_header else 3

    for line in lines[start_idx + 1:]:
        s = line.strip()
        if not s:
            continue

        parts = s.split(",")
        if len(parts) != expected_cols:
            continue

        try:
            [float(x) for x in parts]
            csv_lines.append(s)
        except ValueError:
            continue

    if len(csv_lines) <= 1:
        raise ValueError("The header was found, but no valid samples were found.")

    df = pd.read_csv(io.StringIO("\n".join(csv_lines)))

    if "phi2_bias" not in df.columns:
        df["phi2_bias"] = 1.0

    df = df[["phi0_omega_k", "phi1_u_k", "phi2_bias", "y_omega_k1"]]

    return df


def estimate_parameters(df: pd.DataFrame, Ts: float) -> None:
    """
    Estimates the discrete-time model:

        omega[k+1] = alpha * omega[k] + beta * u[k] + gamma

    and recovers the continuous-time parameters of:

        dot(omega) + a * omega = b * u + c
    """
    Phi = df[["phi0_omega_k", "phi1_u_k", "phi2_bias"]].to_numpy(dtype=float)
    Y = df["y_omega_k1"].to_numpy(dtype=float)

    if len(Phi) < 5:
        raise ValueError("Too few samples to estimate the model.")

    theta, *_ = np.linalg.lstsq(Phi, Y, rcond=None)
    alpha_hat, beta_hat, gamma_hat = theta

    print("=== Estimated discrete parameters ===")
    print(f"alpha_hat = {alpha_hat:.8f}")
    print(f"beta_hat  = {beta_hat:.8f}")
    print(f"gamma_hat = {gamma_hat:.8f}")
    print()

    y_hat = Phi @ theta
    err = Y - y_hat
    rmse = np.sqrt(np.mean(err ** 2))

    ss_res = np.sum(err ** 2)
    ss_tot = np.sum((Y - np.mean(Y)) ** 2)
    r2 = 1.0 - (ss_res / ss_tot) if ss_tot > 0.0 else float("nan")

    print("=== Fit quality ===")
    print(f"Samples   = {len(Y)}")
    print(f"RMSE      = {rmse:.8f}")
    print(f"R^2       = {r2:.8f}")
    print()

    if not (0.0 < alpha_hat < 1.0):
        print(
            "Warning: alpha_hat is outside the expected range (0, 1).\n"
            "That usually indicates sign issues, saturation, poor excitation,\n"
            "heavy noise, or inconsistent data."
        )
        return

    one_minus_alpha = 1.0 - alpha_hat
    if abs(one_minus_alpha) < 1e-12:
        print("Warning: alpha_hat is too close to 1.0, cannot recover continuous parameters reliably.")
        return

    a_hat = -np.log(alpha_hat) / Ts
    b_hat = a_hat * beta_hat / one_minus_alpha
    c_hat = a_hat * gamma_hat / one_minus_alpha

    tau_hat = 1.0 / a_hat
    K_hat = b_hat / a_hat
    d_hat = c_hat / a_hat 

    print("=== Estimated continuous parameters ===")
    print(f"Ts       = {Ts:.6f} s")
    print(f"a_hat    = {a_hat:.8f} 1/s")
    print(f"b_hat    = {b_hat:.8f}")
    print(f"c_hat    = {c_hat:.8f}")
    print(f"tau_hat  = {tau_hat:.8f} s")
    print(f"K_hat    = {K_hat:.8f} rad/s/u")
    print(f"d_hat    = {d_hat:.8f} rad/s")
    print()

    print("=== Model summary ===")
    print("Discrete:")
    print(f"  omega[k+1] = {alpha_hat:.8f} * omega[k] + {beta_hat:.8f} * u[k] + {gamma_hat:.8f}")
    print()
    print("Continuous:")
    print(f"  dot(omega) + {a_hat:.8f} * omega = {b_hat:.8f} * u + {c_hat:.8f}")


def main():
    if len(sys.argv) not in (2, 3):
        print("Usage:")
        print("  python offline_estimation.py <log_file>")
        print("  python offline_estimation.py <log_file> <Ts>")
        print()
        print("Examples:")
        print("  python offline_estimation.py main/output.txt")
        print("  python offline_estimation.py main/output.txt 0.02")
        raise SystemExit(1)

    path = sys.argv[1]
    Ts = float(sys.argv[2]) if len(sys.argv) == 3 else DEFAULT_TS

    df = load_dataset(path)
    estimate_parameters(df, Ts)


if __name__ == "__main__":
    main()