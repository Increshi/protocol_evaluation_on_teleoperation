import argparse
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scapy.all import rdpcap, IP

def print_statistics(data, title, unit):
    """
    Calculates and prints statistical values in a tabular format.
    """
    if len(data) == 0:
        return
    
    df = pd.DataFrame(data, columns=[title])
    
    # Calculate statistics
    stats = {
        "Count": len(data),
        f"Mean ({unit})": np.mean(data),
        f"Std Dev ({unit})": np.std(data),
        f"Min ({unit})": np.min(data),
        f"25th %ile ({unit})": np.percentile(data, 25),
        f"Median / 50th %ile ({unit})": np.median(data),
        f"75th %ile ({unit})": np.percentile(data, 75),
        f"90th %ile ({unit})": np.percentile(data, 90),
        f"95th %ile ({unit})": np.percentile(data, 95),
        f"99th %ile ({unit})": np.percentile(data, 99),
        f"Max ({unit})": np.max(data)
    }
    
    print(f"\n{'='*40}")
    print(f"{title.upper()} STATISTICS")
    print(f"{'='*40}")
    
    # Create a nice looking table
    stats_df = pd.DataFrame(list(stats.items()), columns=['Metric', 'Value'])
    
    # Format float values for better readability 
    def format_val(x):
        if isinstance(x, float):
            return f"{x:.4f}"
        return str(x)
        
    stats_df['Value'] = stats_df['Value'].apply(format_val)
    print(stats_df.to_string(index=False, justify='left'))
    print(f"{'='*40}\n")

def plot_metrics(data, title, xlabel, filename):
    """
    Generates and saves a figure with both a Histogram and a CDF plot.
    """
    data = np.array(data)
    if len(data) == 0:
        print(f"Warning: No data available for {title}. Skipping plots.")
        return

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    # 1. Histogram Plot
    axes[0].hist(data, bins=50, color='skyblue', edgecolor='black')
    axes[0].set_title(f'Histogram: {title}')
    axes[0].set_xlabel(xlabel)
    axes[0].set_ylabel('Frequency (Number of Packets)')
    axes[0].grid(axis='y', alpha=0.75)

    # 2. CDF Plot
    # Sort the data
    data_sorted = np.sort(data)
    # Calculate the proportional values of samples
    p = 1. * np.arange(len(data)) / (len(data) - 1)

    axes[1].plot(data_sorted, p, color='blue', linewidth=2)
    axes[1].set_title(f'CDF: {title}')
    axes[1].set_xlabel(xlabel)
    axes[1].set_ylabel('CDF (Probability)')
    axes[1].grid(True)
    
    # Fill under the CDF curve for better visibility
    axes[1].fill_between(data_sorted, p, color='blue', alpha=0.1)

    plt.tight_layout()
    plt.savefig(filename)
    print(f"✅ Saved plots to: {filename}")
    plt.close()

def main():
    parser = argparse.ArgumentParser(description="Analyze Inter-Arrival Time (IAT) and Packet Sizes from a PCAP file.")
    parser.add_argument("pcap_file", help="Path to the captured .pcap file")
    parser.add_argument("src_ip", help="Source IP address to track")
    parser.add_argument("dst_ip", help="Destination IP address to track")
    args = parser.parse_args()

    print(f"Reading packet capture: {args.pcap_file}...")
    try:
        packets = rdpcap(args.pcap_file)
    except FileNotFoundError:
        print(f"❌ Error: Could not find file {args.pcap_file}")
        return

    timestamps = []
    sizes = []

    # Extract matching packets
    for pkt in packets:
        if IP in pkt:
            if pkt[IP].src == args.src_ip and pkt[IP].dst == args.dst_ip:
                timestamps.append(float(pkt.time))
                sizes.append(len(pkt))

    print(f"Found {len(timestamps)} packets matching the flow {args.src_ip} -> {args.dst_ip}.")

    if len(timestamps) < 2:
        print("❌ Not enough packets found to calculate Inter-Arrival Time (need at least 2).")
        return

    # Sort timestamps just in case the PCAP was captured slightly out of order
    timestamps.sort()

    # Calculate IAT (Inter-Arrival Time) in milliseconds
    # IAT = Time of Current Packet - Time of Previous Packet
    iats_ms = [(timestamps[i] - timestamps[i-1]) * 1000.0 for i in range(1, len(timestamps))]

    # Print Statistics Tables
    print_statistics(iats_ms, "Inter-Arrival Time (IAT)", "ms")
    print_statistics(sizes, "Packet Size", "Bytes")

    # Generate IAT Plots
    plot_metrics(
        data=iats_ms,
        title="Inter-Arrival Time (IAT)",
        xlabel="Time (Milliseconds)",
        filename="iat_plots.png"
    )

    # Generate Packet Size Plots
    plot_metrics(
        data=sizes,
        title="Packet Size",
        xlabel="Size (Bytes)",
        filename="packet_size_plots.png"
    )

if __name__ == "__main__":
    main()
