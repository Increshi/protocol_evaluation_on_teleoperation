# scripts/allPlots.py

import argparse
from pathlib import Path
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

# ============================================================
# CONFIG
# ============================================================

OUTLIER_THRESHOLD = 2
DPI = 400

# ============================================================
# HELPERS
# ============================================================

def find_file(exp_path, keyword):
    files = list(exp_path.glob(f"*{keyword}*.csv"))

    if not files:
        raise FileNotFoundError(
            f"No file containing '{keyword}' found in {exp_path}"
        )

    return files[0]


def optional_find_file(exp_path, keyword):
    files = list(exp_path.glob(f"*{keyword}*.csv"))

    if not files:
        return None

    return files[0]


def save_plot(fig, save_path):
    fig.savefig(save_path, dpi=DPI, bbox_inches="tight")
    plt.close(fig)
    print(f"[SAVED] {save_path}")


def standardize_columns(df):

    if "Processing_ns" in df.columns:
        df["Processing_ms"] = df["Processing_ns"]

    return df


# ============================================================
# LOADERS
# ============================================================

def load_latency_log(exp_path):

    latency_file = find_file(exp_path, "latency")

    df = pd.read_csv(latency_file)

    if df.empty:
        raise ValueError(f"Latency log is empty: {latency_file}")

    df = standardize_columns(df)

    if "RTT_ms" in df.columns:
        df = df[df["RTT_ms"] < 1000].copy()

    df.reset_index(drop=True, inplace=True)

    if "T1_ms" in df.columns:
        t0 = df["T1_ms"].iloc[0]
        df["elapsed_s"] = (df["T1_ms"] - t0) / 1000.0

    elif "T0_ms" in df.columns:
        t0 = df["T0_ms"].iloc[0]
        df["elapsed_s"] = (df["T0_ms"] - t0) / 1000.0

    else:
        df["elapsed_s"] = np.arange(len(df))

    return df


def load_queue_log(exp_path):

    queue_file = find_file(exp_path, "received_queue")

    df = pd.read_csv(queue_file)

    if df.empty:
        raise ValueError(f"Queue log is empty: {queue_file}")

    return df


def load_packet_log(exp_path):

    packet_file = find_file(exp_path, "unity_packet")

    df = pd.read_csv(packet_file)

    if df.empty:
        raise ValueError(f"Packet log is empty: {packet_file}")

    return df


def load_drift_log(exp_path):

    drift_file = find_file(exp_path, "drift")

    df = pd.read_csv(drift_file)

    if df.empty:
        raise ValueError(f"Drift log is empty: {drift_file}")

    return df


# ============================================================
# LATENCY PLOTS
# ============================================================

def plot_latency_timeseries(df, result_dir):

    panels = [

        dict(
            col="RTT_ms",
            label="Control Loop Latency (CLL) (ms)",
            color="#5da9d5",
            title="Control Loop Latency (CLL) Over Time",
            file="cll_vs_time.png"
        ),

        dict(
            col="Processing_ms",
            label="Processing Delay (ms)",
            color="#e08c39",
            title="Processing Delay Over Time",
            file="processing_delay_vs_time.png"
        ),

        dict(
            col="Network_ms",
            label="Network RTT (ms)",
            color="#56b356",
            title="Network RTT Over Time",
            file="network_rtt_vs_time.png"
        ),

        dict(
            col="RenderDelay_ms",
            label="Rendering Delay (ms)",
            color="#d14f4f",
            title="Rendering Delay Over Time",
            file="rendering_delay_vs_time.png"
        ),

        dict(
            col="MTP_ms",
            label="Motion-to-Photon Delay (ms)",
            color="#9d7dbf",
            title="Motion-to-Photon Delay Over Time",
            file="mtp_vs_time.png"
        ),

        dict(
            col="QueueDelay_ms",
            label="AppBuffer Delay (ms)",
            color="#9d7dff",
            title="AppBuffer Delay Over Time",
            file="appbuffer_delay_vs_time.png"
        ),

        dict(
            col="Jitter_ms",
            label="Jitter (ms)",
            color="#f2ba49",
            title="Jitter Over Time",
            file="jitter_vs_time.png"
        ),
    ]

    x = df["elapsed_s"].values

    for p in panels:

        if p["col"] not in df.columns:
            print(f"[SKIP] Missing column: {p['col']}")
            continue

        fig, ax = plt.subplots(figsize=(14, 5))

        y = df[p["col"]].fillna(0).values

        mean = y.mean()
        std = y.std()

        is_out = y > mean + OUTLIER_THRESHOLD * std

        ax.plot(
            x,
            y,
            color=p["color"],
            linewidth=1.5,
            alpha=0.85,
            zorder=2,
            label="Data"
        )

        ax.scatter(
            x[~is_out],
            y[~is_out],
            color=p["color"],
            s=12,
            zorder=3,
            alpha=0.7,
            label="Normal points"
        )

        if is_out.any():

            ax.scatter(
                x[is_out],
                y[is_out],
                color="red",
                s=40,
                zorder=5,
                label="Outliers"
            )

        ax.axhline(
            mean,
            color="navy",
            linewidth=1.2,
            linestyle="--",
            zorder=4,
            label=f"Mean {mean:.2f} ms"
        )

        ax.legend(loc="upper right", fontsize=10)

        ax.set_title(
            p["title"],
            fontsize=14,
            fontweight="bold"
        )

        ax.set_xlabel(
            "Elapsed Time (seconds)",
            fontsize=12
        )

        ax.set_ylabel(
            p["label"],
            fontsize=12
        )

        ax.grid(True, alpha=0.3)

        ax.yaxis.set_major_locator(
            ticker.MaxNLocator(nbins=5)
        )

        save_plot(
            fig,
            result_dir / p["file"]
        )


# ============================================================
# CDF PLOTS
# ============================================================

def plot_cdf(df, result_dir):

    metrics = [
        ("RTT_ms", "Control Loop Latency (CLL)"),
        ("Processing_ms", "Processing Delay"),
        ("Network_ms", "Network RTT"),
        ("RenderDelay_ms", "Rendering Delay"),
        ("MTP_ms", "Motion-to-Photon Delay (MTP)"),
        ("QueueDelay_ms", "AppBuffer Delay"),
        ("Jitter_ms", "Jitter")
    ]

    for col, label in metrics:

        if col not in df.columns:
            continue

        data = df[col].dropna()

        sorted_data = np.sort(data)

        cdf = np.arange(
            1,
            len(sorted_data) + 1
        ) / len(sorted_data)

        fig, ax = plt.subplots(figsize=(10, 5))

        ax.plot(
            sorted_data,
            cdf,
            linewidth=2
        )

        ax.set_title(
            f"CDF of {label}",
            fontsize=14,
            fontweight="bold"
        )

        ax.set_xlabel(
            f"{label} (ms)"
        )

        ax.set_ylabel(
            "Cumulative Probability"
        )

        ax.grid(True, alpha=0.3)

        save_plot(
            fig,
            result_dir / f"{col}_cdf.png"
        )


# ============================================================
# DISTRIBUTION PLOTS
# ============================================================

def plot_distributions(df, result_dir):

    metrics = [
        ("RTT_ms", "Control Loop Latency (CLL)"),
        ("Processing_ms", "Processing Delay"),
        ("Network_ms", "Network RTT"),
        ("RenderDelay_ms", "Rendering Delay"),
        ("MTP_ms", "Motion-to-Photon Delay (MTP)"),
        ("QueueDelay_ms", "AppBuffer Delay"),
        ("Jitter_ms", "Jitter")
    ]

    for col, label in metrics:

        if col not in df.columns:
            continue

        fig, ax = plt.subplots(figsize=(12, 5))

        data = df[col].dropna()

        ax.hist(
            data,
            bins=50,
            alpha=0.8
        )

        ax.set_title(
            f"Distribution of {label}",
            fontsize=14,
            fontweight="bold"
        )

        ax.set_xlabel(
            f"{label} (ms)"
        )

        ax.set_ylabel(
            "Count"
        )

        ax.grid(True, alpha=0.3)

        save_plot(
            fig,
            result_dir / f"{col}_distribution.png"
        )


# ============================================================
# STALL ANALYSIS
# ============================================================

def plot_stalls(queue_df, result_dir):

    ts_col = "unix_ms"
    queue_col = "received_queue_size"

    queue_df = queue_df.sort_values(ts_col)

    mask = queue_df[queue_col] > 0

    last_nonzero = queue_df[ts_col].where(mask).ffill()

    diff = queue_df[ts_col] - last_nonzero

    y = np.where(mask, 0, diff)

    fig, ax = plt.subplots(figsize=(12, 5))

    ax.plot(
        queue_df[ts_col],
        y,
        linewidth=1.5
    )

    ax.set_title(
        "AppBuffer Stall Time",
        fontsize=14,
        fontweight="bold"
    )

    ax.set_xlabel("Time (ms)")
    ax.set_ylabel("Stall Duration (ms)")

    ax.grid(True, alpha=0.3)

    save_plot(
        fig,
        result_dir / "stall_analysis.png"
    )

    x = np.sort(y)
    cdf = np.arange(1, len(x)+1) / len(x)

    fig, ax = plt.subplots(figsize=(8, 5))

    ax.plot(x, cdf)

    ax.set_title(
        "CDF of AppBuffer Stall Durations",
        fontsize=14,
        fontweight="bold"
    )

    ax.set_xlabel("AppBuffer Stall Duration (ms)")
    ax.set_ylabel("CDF")

    ax.grid(True, alpha=0.3)

    save_plot(
        fig,
        result_dir / "stall_cdf.png"
    )


# ============================================================
# DRIFT ANALYSIS
# ============================================================

def plot_drift_analysis(df, result_dir):

    time_col = df.columns[0]
    expected_col = df.columns[1]
    actual_col = df.columns[2]

    time_data = df[time_col]
    expected_pos = df[expected_col]
    actual_pos = df[actual_col]

    drift_error = expected_pos - actual_pos

    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    axes[0].plot(
        time_data,
        expected_pos,
        label="Expected (Unity Client)",
        color='blue',
        linestyle='--',
        linewidth=2
    )

    axes[0].plot(
        time_data,
        actual_pos,
        label="Actual (Isaac Sim)",
        color='cyan',
        linewidth=2
    )

    axes[0].fill_between(
        time_data,
        expected_pos,
        actual_pos,
        color='red',
        alpha=0.15,
        label="Drift Region"
    )

    axes[0].set_title(
        'Robotic Drift: Expected vs Actual Trajectory'
    )

    axes[0].set_ylabel(
        'Joint Angle (Radians)'
    )

    axes[0].legend(loc="upper left")

    axes[0].grid(True, linestyle=':', alpha=0.6)

    axes[1].plot(
        time_data,
        drift_error,
        color='darkorange',
        linewidth=2,
        label="Positional Error"
    )

    axes[1].fill_between(
        time_data,
        0,
        drift_error,
        color='darkorange',
        alpha=0.3
    )

    axes[1].set_title(
        'Cumulative Positional Error (Drift)'
    )

    axes[1].set_xlabel(
        'Time (Seconds)'
    )

    axes[1].set_ylabel(
        'Error (Radians)'
    )

    axes[1].legend(loc="upper left")

    axes[1].grid(True, linestyle=':', alpha=0.6)

    save_plot(
        fig,
        result_dir / "drift_analysis.png"
    )

    final_error = drift_error.iloc[-1]
    max_error = drift_error.max()

    summary_path = result_dir / "drift_summary.txt"

    with open(summary_path, "w") as f:

        f.write("DRIFT ANALYSIS SUMMARY\n")
        f.write("=" * 50 + "\n\n")

        f.write(
            f"Total Commands Expected: "
            f"{expected_pos.iloc[-1]:.3f} rads\n"
        )

        f.write(
            f"Total Commands Executed: "
            f"{actual_pos.iloc[-1]:.3f} rads\n"
        )

        expected_total = (
            expected_pos.iloc[-1]
            if expected_pos.iloc[-1] != 0
            else 1
        )

        f.write(
            f"Final Drift Error: "
            f"{final_error:.3f} rads "
            f"({(final_error/expected_total)*100:.1f}% loss)\n"
        )

        f.write(
            f"Maximum Error Recorded: "
            f"{max_error:.3f} rads\n"
        )

    print(f"[SAVED] {summary_path}")


# ============================================================
# WEBRTC THROUGHPUT ANALYSIS
# ============================================================

def plot_throughput_analysis(stats_df, result_dir):

    stats_df['timestamp_utc'] = pd.to_datetime(
        stats_df['timestamp_utc']
    )

    df_robot = stats_df[
        stats_df['channel_label'] == 'robot_state'
    ]

    df_cmd = stats_df[
        stats_df['channel_label'] == 'commands'
    ]

    fig, axes = plt.subplots(
        2,
        1,
        figsize=(10, 8),
        sharex=True
    )

    axes[0].plot(
        df_robot['timestamp_utc'],
        df_robot['throughput_send_kbps'],
        label="Robot State (Send)",
        color='salmon',
        linestyle='--'
    )

    axes[0].plot(
        df_robot['timestamp_utc'],
        df_robot['throughput_recv_kbps'],
        label="Robot State (Receive)",
        color='blue',
        linewidth=2
    )

    axes[0].set_title(
        'Robot State Channel Throughput'
    )

    axes[0].set_ylabel(
        'Throughput (kbps)'
    )

    axes[0].legend(loc="upper left")

    axes[0].grid(True, linestyle=':', alpha=0.6)

    axes[1].plot(
        df_cmd['timestamp_utc'],
        df_cmd['throughput_send_kbps'],
        label="Commands (Send)",
        color='red',
        linewidth=2
    )

    axes[1].plot(
        df_cmd['timestamp_utc'],
        df_cmd['throughput_recv_kbps'],
        label="Commands (Receive)",
        color='lightblue',
        linestyle='--'
    )

    axes[1].set_title(
        'Commands Channel Throughput'
    )

    axes[1].set_xlabel(
        'Time (UTC)'
    )

    axes[1].set_ylabel(
        'Throughput (kbps)'
    )

    axes[1].legend(loc="upper left")

    axes[1].grid(True, linestyle=':', alpha=0.6)

    fig.autofmt_xdate()

    save_plot(
        fig,
        result_dir / "throughput_analysis.png"
    )


# ============================================================
# PACKET LOSS
# ============================================================

def analyze_packet_loss(packet_df, result_dir):

    possible_cols = [
        "seq",
        "cmd_seq",
        "cmd_seq_echo"
    ]

    seq_col = None

    for c in possible_cols:
        if c in packet_df.columns:
            seq_col = c
            break

    if seq_col is None:
        print("[SKIP] No sequence column found")
        return

    cmd = pd.to_numeric(
        packet_df[seq_col],
        errors="coerce"
    ).dropna().astype(int)

    observed = set(cmd.unique())

    expected = set(
        range(cmd.min(), cmd.max() + 1)
    )

    missing = expected - observed

    missing_count = len(missing)

    expected_count = len(expected)

    missing_pct = (
        missing_count / expected_count
    ) * 100

    summary_path = result_dir / "packet_loss_summary.txt"

    with open(summary_path, "w") as f:

        f.write(f"Sequence Column: {seq_col}\n")
        f.write(f"Expected Count: {expected_count}\n")
        f.write(f"Observed Count: {len(observed)}\n")
        f.write(f"Missing Count: {missing_count}\n")
        f.write(f"Packet Loss Percentage: {missing_pct:.4f}%\n")

        if missing_count:
            f.write(
                f"\nFirst Missing Sequences:\n"
            )

            f.write(
                str(sorted(missing)[:50])
            )

    print(f"[SAVED] {summary_path}")


# ============================================================
# MAIN
# ============================================================

def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--client_exp",
        required=True,
        help="Client experiment directory"
    )

    parser.add_argument(
        "--server_exp",
        required=False,
        help="Server experiment directory"
    )

    args = parser.parse_args()

    client_exp = Path(args.client_exp)

    result_dir = client_exp / "resultPlots"

    result_dir.mkdir(exist_ok=True)

    print(f"\n[PROCESSING] {client_exp}")

    latency_df = load_latency_log(client_exp)

    queue_df = load_queue_log(client_exp)

    packet_df = load_packet_log(client_exp)

    drift_df = load_drift_log(client_exp)

    plot_latency_timeseries(
        latency_df,
        result_dir
    )

    plot_cdf(
        latency_df,
        result_dir
    )

    plot_distributions(
        latency_df,
        result_dir
    )

    plot_stalls(
        queue_df,
        result_dir
    )

    plot_drift_analysis(
        drift_df,
        result_dir
    )

    analyze_packet_loss(
        packet_df,
        result_dir
    )

    # --------------------------------------------------------
    # WEBRTC THROUGHPUT
    # --------------------------------------------------------

    webrtc_stats_file = optional_find_file(
        client_exp,
        "webrtc_channel_stats"
    )

    if webrtc_stats_file is not None:

        stats_df = pd.read_csv(
            webrtc_stats_file
        )

        plot_throughput_analysis(
            stats_df,
            result_dir
        )

    print("\n[DONE]")


if __name__ == "__main__":
    main()