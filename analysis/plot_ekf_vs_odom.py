import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np


def plot_ekf_vs_odom_error(csv_path: str, save_path: str = None):
    """
    Plot maintained EKF vs odometry position/heading errors over time.
    """
    df = pd.read_csv(csv_path)

    # convert microsecond timestamps to seconds relative to start
    t = (df["ts_microsecs"] - df["ts_microsecs"].iloc[0]) / 1e6

    # Compute 2D position error magnitude for convenience
    df["ekf_pos_err"] = np.hypot(df["error_x"], df["error_z"])  # hypot computes distances
    df["odom_pos_err"] = np.hypot(df["odom_err_x"], df["odom_err_z"])

    fig = plt.figure(figsize=(14, 10))
    gs = gridspec.GridSpec(3, 1, hspace=0.45)

    # x-error
    ax0 = fig.add_subplot(gs[0])
    ax0.plot(t, df["error_x"], label="EKF", color="#2196F3", linewidth=1.2)
    ax0.plot(t, df["odom_err_x"], label="Odom", color="#FF5722", linewidth=1.2, alpha=0.8)
    ax0.axhline(0, color="gray", linewidth=0.6, linestyle="--")
    ax0.set_ylabel("Error X (m)")
    ax0.set_title("X-axis Error")
    ax0.legend(loc="upper right")
    ax0.grid(True, alpha=0.3)

    # z-error
    ax1 = fig.add_subplot(gs[1], sharex=ax0)
    ax1.plot(t, df["error_z"],    label="EKF",  color="#2196F3", linewidth=1.2)
    ax1.plot(t, df["odom_err_z"], label="Odom", color="#FF5722", linewidth=1.2, alpha=0.8)
    ax1.axhline(0, color="gray", linewidth=0.6, linestyle="--")
    ax1.set_ylabel("Error Z (m)")
    ax1.set_title("Z-axis Error")
    ax1.legend(loc="upper right")
    ax1.grid(True, alpha=0.3)

    # header-error
    ax2 = fig.add_subplot(gs[2], sharex=ax0)
    ax2.plot(t, df["error_heading"],    label="EKF",  color="#2196F3", linewidth=1.2)
    ax2.plot(t, df["odom_err_heading"], label="Odom", color="#FF5722", linewidth=1.2, alpha=0.8)
    ax2.axhline(0, color="gray", linewidth=0.6, linestyle="--")
    ax2.set_ylabel("Error (rad)")
    ax2.set_xlabel("Time (s)")
    ax2.set_title("Heading Error")
    ax2.legend(loc="upper right")
    ax2.grid(True, alpha=0.3)

    # shared time axis, bottom plot for clarity
    plt.setp(ax0.get_xticklabels(), visible=False)
    plt.setp(ax1.get_xticklabels(), visible=False)

    fig.savefig(save_path, dpi=150, bbox_inches="tight")

    return fig


def plot_position_error_magnitude(csv_path: str, save_path: str = None):
    """
    Plot the total 2D Euclidean position error magnitude for EKF vs odometry
    """
    df = pd.read_csv(csv_path)
    t = (df["ts_microsecs"] - df["ts_microsecs"].iloc[0]) / 1e6

    ekf_mag = np.hypot(df["error_x"], df["error_z"])
    odom_mag = np.hypot(df["odom_err_x"], df["odom_err_z"])

    window = max(1, len(df) // 20)  # window contains 5% of the dataset, sufficient smoothing
    ekf_rms = ekf_mag.rolling(window,  center=True).apply(lambda v: np.sqrt(np.mean(v**2)))
    odom_rms = odom_mag.rolling(window, center=True).apply(lambda v: np.sqrt(np.mean(v**2)))

    fig, ax = plt.subplots(figsize=(13, 5))
    ax.plot(t, ekf_mag, color="#2196F3", linewidth=0.8, alpha=0.5, label="EKF magnitude")
    ax.plot(t, odom_mag, color="#FF5722", linewidth=0.8, alpha=0.5, label="Odom magnitude")
    ax.plot(t, ekf_rms, color="#0D47A1", linewidth=2.0, label=f"EKF rolling RMS (w={window})")
    ax.plot(t, odom_rms, color="#BF360C", linewidth=2.0, label=f"Odom rolling RMS (w={window})")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position Error Magnitude (m)")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()

    fig.savefig(save_path, dpi=150, bbox_inches="tight")

    return fig


if __name__ == "__main__":
    import sys
    csv = sys.argv[1]
    plot_ekf_vs_odom_error(csv, save_path="ekf_vs_odom_components.png")
    plot_position_error_magnitude(csv, save_path="ekf_vs_odom_magnitude.png")