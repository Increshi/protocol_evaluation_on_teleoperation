import time
import subprocess

iface = "eno1"
trace_file = "report_bicycle_0002_throughput.txt"

def run(cmd):
    subprocess.run(cmd, shell=True)

# reset tc
run(f"sudo tc qdisc del dev {iface} root 2>/dev/null")

# create root qdisc
run(f"sudo tc qdisc add dev {iface} root handle 1: htb default 1")

# base class
run(f"sudo tc class add dev {iface} parent 1: classid 1:1 htb rate 1000mbit")

print("Starting bandwidth emulation...")

drop_active = False

with open(trace_file) as f:

    for line in f:

        interval, rate = line.split()

        interval = float(interval)
        rate = float(rate)

        # -----------------------------
        # TRUE ZERO THROUGHPUT
        # -----------------------------
        if rate <= 0:

            # activate packet drop only once
            if not drop_active:

                run(
                    f"sudo tc qdisc add dev {iface} parent 1:1 handle 10: netem loss 100%"
                )

                drop_active = True

            print(f"DROP 100% interval={interval} ms")

        # -----------------------------
        # NORMAL THROUGHPUT
        # -----------------------------
        else:

            # remove drop rule if active
            if drop_active:

                run(
                    f"sudo tc qdisc del dev {iface} parent 1:1 handle 10:"
                )

                drop_active = False

            cmd = (
                f"sudo tc class change dev {iface} "
                f"parent 1: classid 1:1 "
                f"htb rate {rate}mbit"
            )

            run(cmd)

            print(f"rate={rate:.2f} Mbps interval={interval} ms")

        time.sleep(interval / 1000.0)

print("Finished replay.")
