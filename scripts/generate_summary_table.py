# scripts/generate_summary_table.py

from pathlib import Path
import pandas as pd
import numpy as np

# ============================================================
# ROOT DIRECTORIES
# ============================================================

CLIENT_ROOT = Path("../Client_Experimentation_final")

# ============================================================
# PROTOCOL MAP
# ============================================================

PROTOCOLS = {

    # ========================================================
    # NORMAL TCP/UDP COMBINATIONS
    # ========================================================

    "Cudp_Sudp": "Cudp_Sudp",
    "Ctcp_Stcp": "Ctcp_Stcp",
    "Ctcp_Sudp": "Ctcp_Sudp",
    "Cudp_Stcp": "Cudp_Stcp",

    # ========================================================
    # WEBRTC COMBINATIONS
    # ========================================================

    "WebRTC/Cudp_Sudp": "WebRTC_Cudp_Sudp",
    "WebRTC/Ctcp_Stcp": "WebRTC_Ctcp_Stcp",
    "WebRTC/Ctcp_Sudp": "WebRTC_Ctcp_Sudp",
    "WebRTC/Cudp_Stcp": "WebRTC_Cudp_Stcp"
}

VARIANTS = [
    "BaseLine",
    "Network_Variant1",
    "Network_Variant2"
]

# ============================================================
# METRIC COLUMN MAP
# ============================================================

METRIC_COLUMNS = {
    "Control Loop Latency (CLL)": "RTT_ms",
    "AppBuffer Delay": "QueueDelay_ms",
    "Rendering Delay": "RenderDelay_ms",
    "Processing Delay": "Processing_ms",
    "Network RTT": "Network_ms",
    "Motion-to-Photon Delay (MTP)": "MTP_ms",
    "Jitter": "Jitter_ms"
}

# ============================================================
# HELPERS
# ============================================================

def find_latency_file(exp_path):
    files = list(exp_path.glob("*latency*.csv"))

    if not files:
        return None

    return files[0]


def compute_metric_average(exp_path, metric_column):

    latency_file = find_latency_file(exp_path)

    if latency_file is None:
        return None

    try:

        df = pd.read_csv(latency_file)

        if df.empty:
            return None

        if "Processing_ns" in df.columns:
            df["Processing_ms"] = df["Processing_ns"]

        if metric_column not in df.columns:
            return None

        values = pd.to_numeric(
            df[metric_column],
            errors="coerce"
        ).dropna()

        if len(values) == 0:
            return None

        return values.mean()

    except Exception as e:

        print(f"Error processing {latency_file}")
        print(e)

        return None


# ============================================================
# BUILD SUMMARY TABLE
# ============================================================

summary_rows = []

for metric_name, metric_column in METRIC_COLUMNS.items():

    row = {
        "Metric": metric_name
    }

    for protocol_folder, protocol_display in PROTOCOLS.items():

        for variant in VARIANTS:

            key = f"{protocol_display}_{variant}"

            protocol_path = CLIENT_ROOT / variant / protocol_folder

            if not protocol_path.exists():

                row[key] = "-"
                continue

            experiment_dirs = sorted(
                [
                    p for p in protocol_path.glob("Exp*")
                    if p.is_dir()
                ]
            )

            all_means = []

            for exp_dir in experiment_dirs:

                avg = compute_metric_average(
                    exp_dir,
                    metric_column
                )

                if avg is not None:
                    all_means.append(avg)

            if len(all_means) == 0:

                row[key] = "-"

            else:

                final_avg = np.mean(all_means)

                row[key] = round(final_avg, 2)

    summary_rows.append(row)

# ============================================================
# CREATE DATAFRAME
# ============================================================

summary_df = pd.DataFrame(summary_rows)

# ============================================================
# COLUMN ORDER
# ============================================================

ordered_columns = [

    "Metric",

    # ========================================================
    # NORMAL TRANSPORT
    # ========================================================

    "Cudp_Sudp_BaseLine",
    "Cudp_Sudp_Network_Variant1",
    "Cudp_Sudp_Network_Variant2",

    "Ctcp_Stcp_BaseLine",
    "Ctcp_Stcp_Network_Variant1",
    "Ctcp_Stcp_Network_Variant2",

    "Ctcp_Sudp_BaseLine",
    "Ctcp_Sudp_Network_Variant1",
    "Ctcp_Sudp_Network_Variant2",

    "Cudp_Stcp_BaseLine",
    "Cudp_Stcp_Network_Variant1",
    "Cudp_Stcp_Network_Variant2",

    # ========================================================
    # WEBRTC
    # ========================================================

    "WebRTC_Cudp_Sudp_BaseLine",
    "WebRTC_Cudp_Sudp_Network_Variant1",
    "WebRTC_Cudp_Sudp_Network_Variant2",

    "WebRTC_Ctcp_Stcp_BaseLine",
    "WebRTC_Ctcp_Stcp_Network_Variant1",
    "WebRTC_Ctcp_Stcp_Network_Variant2",

    "WebRTC_Ctcp_Sudp_BaseLine",
    "WebRTC_Ctcp_Sudp_Network_Variant1",
    "WebRTC_Ctcp_Sudp_Network_Variant2",

    "WebRTC_Cudp_Stcp_BaseLine",
    "WebRTC_Cudp_Stcp_Network_Variant1",
    "WebRTC_Cudp_Stcp_Network_Variant2",
]

summary_df = summary_df[ordered_columns]

# ============================================================
# SAVE
# ============================================================

output_csv = "summary_metrics_table.csv"

summary_df.to_csv(output_csv, index=False)

print("\n===================================================")
print("SUMMARY TABLE GENERATED")
print("===================================================")

print(summary_df)

print(f"\nSaved CSV → {output_csv}")