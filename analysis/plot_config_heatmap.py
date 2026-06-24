import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import numpy as np
import sys


def plot_config_heatmaps(csv_path: str, save_path):
    """
    Plot MPPI tuning heatmaps over w_heading vs w_speed with
    mean RMS position error, mean peak position error, mean RMS heading error,
    and mean peak heading error.
    """
    df = pd.read_csv(csv_path)

    speed_vals = sorted(df["speed_noise"].unique())
    angular_vals = sorted(df["angular_noise_deg"].unique())

    metrics = [
        ("mean_rms_pos_err_m", "Mean RMS Position Error (m)", "YlOrRd"),
        ("mean_peak_pos_err_m", "Mean Peak Position Error (m)", "YlOrRd"),
        ("mean_rms_heading_err_deg", "Mean RMS Heading Error (°)", "YlGnBu"),
        ("mean_peak_heading_err_deg","Mean Peak Heading Error (°)", "YlGnBu"),
    ]

    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    axes = axes.flatten()

    for ax, (col, title, cmap) in zip(axes, metrics):
        # fill grid with NaNs for missing values
        grid = np.full((len(speed_vals), len(angular_vals)), np.nan)
        for _, row in df.iterrows():
            r = speed_vals.index(row["speed_noise"])
            c = angular_vals.index(row["angular_noise_deg"])
            grid[r, c] = row[col]

        best_r, best_c = np.unravel_index(np.nanargmin(grid), grid.shape)

        im = ax.imshow(grid, cmap=cmap, aspect="auto",
                       norm=mcolors.Normalize(vmin=np.nanmin(grid),
                                              vmax=np.nanmax(grid)))

        # annotate with vals
        for r in range(len(speed_vals)):
            for c in range(len(angular_vals)):
                val = grid[r, c]
                if not np.isnan(val):
                    txt_color = "white" if val > (np.nanmax(grid) * 0.6) else "black"
                    ax.text(c, r, f"{val:.3f}", ha="center", va="center",
                            fontsize=9, color=txt_color, fontweight="normal")

        # highlight with lime
        ax.add_patch(plt.Rectangle(
            (best_c - 0.5, best_r - 0.5), 1, 1,
            fill=False, edgecolor="lime", linewidth=3, zorder=5
        ))

        ax.set_xticks(range(len(angular_vals)))
        ax.set_xticklabels([f"{v}°" for v in angular_vals])
        ax.set_yticks(range(len(speed_vals)))
        ax.set_yticklabels([str(v) for v in speed_vals])
        ax.set_xlabel("Angular Noise (deg)")
        ax.set_ylabel("Speed Noise")
        ax.set_title(title, fontweight="bold")

        color_bar = fig.colorbar(im, ax=ax, shrink=0.85)
        color_bar.ax.tick_params(labelsize=8)

    fig.tight_layout(rect=[0, 0, 1, 0.95])

    fig.savefig(save_path, dpi=150, bbox_inches="tight")

    return fig


def plot_std_heatmaps(csv_path: str, save_path: str = None):
    df = pd.read_csv(csv_path)

    speed_vals = sorted(df["speed_noise"].unique())
    angular_vals = sorted(df["angular_noise_deg"].unique())
    # speed_vals = sorted(df["lidar_noise"].unique())
    # angular_vals = sorted(df["angle_noise_deg"].unique())

    metrics = [
        ("std_rms_pos_err_m", "Std RMS Position Error (m)", "Blues"),
        ("std_peak_pos_err_m", "Std Peak Position Error (m)", "Blues"),
        ("std_rms_heading_err_deg", "Std RMS Heading Error (°)", "YlGnBu"),
        ("std_peak_heading_err_deg","Std Peak Heading Error (°)", "YlGnBu"),
    ]

    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    axes = axes.flatten()

    for ax, (col, title, cmap) in zip(axes, metrics):
        grid = np.full((len(speed_vals), len(angular_vals)), np.nan)
        for _, row in df.iterrows():
            r = speed_vals.index(row["speed_noise"])
            c = angular_vals.index(row["angular_noise_deg"])
            grid[r, c] = row[col]

        best_r, best_c = np.unravel_index(np.nanargmin(grid), grid.shape)

        im = ax.imshow(grid, cmap=cmap, aspect="auto",
                       norm=mcolors.Normalize(vmin=np.nanmin(grid),
                                              vmax=np.nanmax(grid)))

        for r in range(len(speed_vals)):
            for c in range(len(angular_vals)):
                val = grid[r, c]
                if not np.isnan(val):
                    txt_color = "white" if val > (np.nanmax(grid) * 0.6) else "black"
                    ax.text(c, r, f"{val:.3f}", ha="center", va="center",
                            fontsize=9, color=txt_color)

        ax.add_patch(plt.Rectangle(
            (best_c - 0.5, best_r - 0.5), 1, 1,
            fill=False, edgecolor="lime", linewidth=3, zorder=5
        ))

        ax.set_xticks(range(len(angular_vals)))
        ax.set_xticklabels([f"{v}°" for v in angular_vals])
        ax.set_yticks(range(len(speed_vals)))
        ax.set_yticklabels([str(v) for v in speed_vals])
        ax.set_xlabel("Angular Noise (deg)")
        ax.set_ylabel("Speed Noise")
        ax.set_title(title, fontweight="bold")

        color_bar = fig.colorbar(im, ax=ax, shrink=0.85)
        color_bar.ax.tick_params(labelsize=8)

    fig.tight_layout(rect=[0, 0, 1, 0.95])

    fig.savefig(save_path, dpi=150, bbox_inches="tight")


    return fig


if __name__ == "__main__":
    csv = sys.argv[1]

    plot_config_heatmaps(csv, save_path="heatmap_means.png")
    plot_std_heatmaps(csv, save_path="heatmap_stds.png")