import io
import sys
import numpy as np
import pandas as pd


DEFAULT_TS = 0.02  # 20 ms


def load_dataset(path: str) -> pd.DataFrame:
    """
    Lee un archivo de log y extrae únicamente la tabla CSV con columnas:
        phi0_omega_k,phi1_u_k,y_omega_k1

    Ignora texto basura antes/después, como logs del ESP32 o del monitor serial.
    """
    text = open(path, "r", encoding="utf-8", errors="ignore").read()
    lines = text.splitlines()

    header = "phi0_omega_k,phi1_u_k,y_omega_k1"
    start_idx = None

    for i, line in enumerate(lines):
        if line.strip() == header:
            start_idx = i
            break

    if start_idx is None:
        raise ValueError(
            "No se encontró el encabezado CSV esperado: "
            "'phi0_omega_k,phi1_u_k,y_omega_k1'"
        )

    csv_lines = [header]

    for line in lines[start_idx + 1:]:
        s = line.strip()
        if not s:
            continue

        parts = s.split(",")
        if len(parts) != 3:
            continue

        try:
            float(parts[0])
            float(parts[1])
            float(parts[2])
            csv_lines.append(s)
        except ValueError:
            continue

    if len(csv_lines) <= 1:
        raise ValueError("Se encontró el encabezado, pero no hay muestras válidas.")

    csv_text = "\n".join(csv_lines)
    df = pd.read_csv(io.StringIO(csv_text))
    return df


def estimate_parameters(df: pd.DataFrame, Ts: float) -> None:
    Phi = df[["phi0_omega_k", "phi1_u_k"]].to_numpy(dtype=float)
    Y = df["y_omega_k1"].to_numpy(dtype=float)

    if len(Phi) < 5:
        raise ValueError("Muy pocas muestras para estimar. Necesitas más datos.")

    theta, *_ = np.linalg.lstsq(Phi, Y, rcond=None)
    alpha_hat, beta_hat = theta

    print("=== Estimated discrete parameters ===")
    print(f"alpha_hat = {alpha_hat:.8f}")
    print(f"beta_hat  = {beta_hat:.8f}")
    print()

    y_hat = Phi @ theta
    err = Y - y_hat
    rmse = np.sqrt(np.mean(err ** 2))

    print("=== Fit quality ===")
    print(f"Samples   = {len(Y)}")
    print(f"RMSE      = {rmse:.8f}")
    print()

    if not (0.0 < alpha_hat < 1.0):
        print(
            "Advertencia: alpha_hat está fuera del rango (0,1).\n"
            "Eso suele indicar problemas de signo, saturación, ruido, "
            "poca excitación o datos inconsistentes."
        )
        return

    a_hat = -np.log(alpha_hat) / Ts
    b_hat = a_hat * beta_hat / (1.0 - alpha_hat)
    tau_hat = 1.0 / a_hat
    K_hat = b_hat / a_hat

    print("=== Estimated continuous parameters ===")
    print(f"Ts       = {Ts:.6f} s")
    print(f"a_hat    = {a_hat:.8f} 1/s")
    print(f"b_hat    = {b_hat:.8f}")
    print(f"tau_hat  = {tau_hat:.8f} s")
    print(f"K_hat    = {K_hat:.8f} rad/s/u")


def main():
    if len(sys.argv) not in (2, 3):
        print("Uso:")
        print("  python offline_estimation.py <archivo_log>")
        print("  python offline_estimation.py <archivo_log> <Ts>")
        print()
        print("Ejemplo:")
        print("  python offline_estimation.py main/output.txt")
        print("  python offline_estimation.py main/output.txt 0.02")
        raise SystemExit(1)

    path = sys.argv[1]
    Ts = float(sys.argv[2]) if len(sys.argv) == 3 else DEFAULT_TS

    df = load_dataset(path)
    estimate_parameters(df, Ts)


if __name__ == "__main__":
    main()