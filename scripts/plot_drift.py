import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import argparse
import os

def plot_drift_analysis(csv_file, output_filename="drift_analysis.png"):
    """
    Plots the expected vs actual trajectory to visualize robotic drift.
    Expected CSV format: Time, ExpectedPos, ActualPos
    """
    if not os.path.exists(csv_file):
        print(f"❌ Could not find {csv_file}")
        return

    # Load data
    df = pd.read_csv(csv_file)
    
    # Ensure columns exist (fallback to index if headers differ slightly)
    time_col = df.columns[0]
    expected_col = df.columns[1]
    actual_col = df.columns[2]

    time_data = df[time_col]
    expected_pos = df[expected_col]
    actual_pos = df[actual_col]

    # Calculate Drift Error (Expected - Actual)
    drift_error = expected_pos - actual_pos

    # Setup the plot
    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    # --- Plot 1: Trajectory Divergence ---
    axes[0].plot(time_data, expected_pos, label="Expected (Unity Client)", color='blue', linestyle='--', linewidth=2)
    axes[0].plot(time_data, actual_pos, label="Actual (Isaac Sim)", color='red', linewidth=2)
    
    # Shade the area between the curves to highlight the drift
    axes[0].fill_between(time_data, expected_pos, actual_pos, color='red', alpha=0.15, label="Missing Movement (Lost Packets)")
    
    axes[0].set_title('Robotic Drift: Expected vs. Actual Trajectory (Delta Commands over UDP/Unreliable)')
    axes[0].set_ylabel('Joint Angle (Radians)')
    axes[0].legend(loc="upper left")
    axes[0].grid(True, linestyle=':', alpha=0.6)

    # --- Plot 2: Accumulated Error ---
    axes[1].plot(time_data, drift_error, color='darkorange', linewidth=2, label="Positional Error")
    axes[1].fill_between(time_data, 0, drift_error, color='darkorange', alpha=0.3)
    
    axes[1].set_title('Cumulative Positional Error (Drift)')
    axes[1].set_xlabel('Time (Seconds)')
    axes[1].set_ylabel('Error (Radians)')
    axes[1].legend(loc="upper left")
    axes[1].grid(True, linestyle=':', alpha=0.6)

    # Finalize and Save
    plt.tight_layout()
    plt.savefig(output_filename, dpi=300)
    print(f"✅ Saved Drift Analytics plot to: {output_filename}")
    
    # Print quantitative results
    final_error = drift_error.iloc[-1]
    max_error = drift_error.max()
    print("\n--- QUANTITATIVE RESULTS ---")
    print(f"Total Commands Expected: {expected_pos.iloc[-1]:.3f} rads")
    print(f"Total Commands Executed: {actual_pos.iloc[-1]:.3f} rads")
    print(f"Final Drift Error:       {final_error:.3f} rads ({(final_error/expected_pos.iloc[-1])*100:.1f}% loss)")
    print(f"Maximum Error Recorded:  {max_error:.3f} rads")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Analyze and plot Robotic Drift from a CSV file.")
    parser.add_argument("csv_file", help="Path to your drift_log.csv (Columns: Time, ExpectedPos, ActualPos)")
    args = parser.parse_args()
    
    plot_drift_analysis(args.csv_file)
