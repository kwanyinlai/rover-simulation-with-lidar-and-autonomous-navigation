import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import numpy as np
import sys


def load_and_aggregate(csv_path: str) -> pd.DataFrame:
    """
    Load raw per-run MPPI results and aggregate by config_id.
    DNF runs are excluded from mean completion time but counted separately.
    """
    df = pd.read_csv(csv_path)

    df["dnf"] = df["completion_time_s"] == "DNF" # new DNF row
    df["completion_time_s"] = pd.to_numeric(df["completion_time_s"], errors="coerce")
    # turn DNFs to NaN completion times

    aggregate = df.groupby("config_id").apply(lambda config_run: pd.Series({
        "w_heading": config_run["w_heading"].iloc[0],
        "w_speed": config_run["w_speed"].iloc[0],
        "repeats": len(config_run),
        "dnf_count": config_run["dnf"].sum(),
        "dnf_rate": config_run["dnf"].mean(),
        "mean_completion_s": config_run["completion_time_s"].mean(), # NaN safe to exclude DNFs turned to NaNs
        "std_completion_s": config_run["completion_time_s"].std(),
    })).reset_index()

    return aggregate


def plot_mppi_heatmaps(csv_path: str):
    """
    Plot MPPI tuning heatmaps over w_heading vs w_speed with
    mean completion time, std completion time, and DNF count/rate
    """
    df = load_and_aggregate(csv_path)

    heading_vals = sorted(df["w_heading"].unique())
    speed_vals = sorted(df["w_speed"].unique())

    metrics = [
        ("mean_completion_s", "Mean Completion Time (s)", "YlOrRd", True),
        ("dnf_rate", "DNF Rate", "Reds", True),
        ("std_completion_s", "Std Completion Time (s)", "Oranges", True),
        ("dnf_count", "DNF Count", "Reds", True),
    ]

    fig, axes = plt.subplots(2, 2, figsize=(13, 9))

    # flatten then zip with metrics
    axes = axes.flatten()
    for ax, (col, title, cmap, is_lower_val_better) in zip(axes, metrics):
        
        grid = np.full((len(heading_vals), len(speed_vals)), np.nan)

        for _, row in df.iterrows():
            r = heading_vals.index(row["w_heading"])
            c = speed_vals.index(row["w_speed"])
            grid[r, c] = row[col]

        if is_lower_val_better:
            best_idx = np.unravel_index(np.nanargmin(grid), grid.shape)
        else:
            best_idx = np.unravel_index(np.nanargmax(grid), grid.shape)

        # colour range
        vmin = np.nanmin(grid)
        vmax = np.nanmax(grid)
        norm = mcolors.Normalize(vmin=vmin, vmax=vmax) if vmin != vmax else None

        im = ax.imshow(grid, cmap=cmap, aspect="auto", norm=norm)

        # annotate cells
        for r in range(len(heading_vals)):
            for c in range(len(speed_vals)):
                val = grid[r, c]
                if np.isnan(val):
                    label = "N/A"
                    txt_color = "gray"
                elif col == "dnf_rate":
                    label = f"{val:.0%}"
                    txt_color = "white" if val > 0.5 else "black"
                elif col == "dnf_count":
                    label = f"{int(val)}"
                    txt_color = "white" if val >= 2 else "black"
                else:
                    label = f"{val:.2f}"
                    txt_color = "white" if (vmax > vmin and val > vmin + 0.6*(vmax-vmin)) else "black"
                ax.text(c, r, label, ha="center", va="center", fontsize=10,
                        color=txt_color, fontweight="normal")

        # lime border on best cell
        br, bc = best_idx
        ax.add_patch(plt.Rectangle(
            (bc - 0.5, br - 0.5), 1, 1,
            fill=False, edgecolor="lime", linewidth=3, zorder=5
        ))

        ax.set_xticks(range(len(speed_vals)))
        ax.set_xticklabels([str(v) for v in speed_vals])
        ax.set_yticks(range(len(heading_vals)))
        ax.set_yticklabels([str(v) for v in heading_vals])
        ax.set_xlabel("w_speed")
        ax.set_ylabel("w_heading")
        ax.set_title(title, fontweight="bold")

        fig.colorbar(im, ax=ax, shrink=0.85).ax.tick_params(labelsize=8)

    fig.tight_layout(rect=[0, 0, 1, 0.95])

    return fig


if __name__ == "__main__":
    csv = sys.argv[1]
    plot_mppi_heatmaps(csv, save_path="heatmap_mppi.png")