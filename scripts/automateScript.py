# scripts/automateScript.py

from pathlib import Path
import subprocess

# ============================================================
# ROOT DIRECTORIES
# ============================================================

CLIENT_ROOT = Path("../Client_Experimentation_final")
SERVER_ROOT = Path("../Server_Experimentation_final")

ALL_PLOTS_SCRIPT = Path("allPlots.py")

# ============================================================
# FIND ALL EXPERIMENTS
# ============================================================

client_experiments = sorted(
    [p for p in CLIENT_ROOT.rglob("Exp*") if p.is_dir()]
)

print(f"\nFound {len(client_experiments)} experiments.\n")

# ============================================================
# RUN ANALYSIS
# ============================================================

successful = []
failed = []

for client_exp in client_experiments:

    relative_path = client_exp.relative_to(CLIENT_ROOT)

    server_exp = SERVER_ROOT / relative_path

    print("=" * 80)
    print(f"PROCESSING: {relative_path}")
    print("=" * 80)

    # --------------------------------------------------------

    if not server_exp.exists():

        print(f"[ERROR] Missing server path:")
        print(server_exp)

        failed.append(str(relative_path))

        continue

    # --------------------------------------------------------

    try:

        result = subprocess.run(
            [
                "python3",
                str(ALL_PLOTS_SCRIPT),
                "--client_exp",
                str(client_exp),
                "--server_exp",
                str(server_exp)
            ],
            capture_output=True,
            text=True
        )

        print(result.stdout)

        if result.returncode != 0:

            print("[FAILED]")
            print(result.stderr)

            failed.append(str(relative_path))

        else:

            print("[SUCCESS]")
            successful.append(str(relative_path))

    except Exception as e:

        print(f"[EXCEPTION] {e}")

        failed.append(str(relative_path))

# ============================================================
# FINAL SUMMARY
# ============================================================

print("\n" + "=" * 100)
print("FINAL SUMMARY")
print("=" * 100)

print(f"\nSuccessful Experiments: {len(successful)}")
print(f"Failed Experiments    : {len(failed)}")

# ------------------------------------------------------------

if successful:

    print("\nSUCCESSFUL:")
    for s in successful:
        print(f"  - {s}")

# ------------------------------------------------------------

if failed:

    print("\nFAILED:")
    for f in failed:
        print(f"  - {f}")

# ============================================================
# SAVE SUMMARY
# ============================================================

summary_path = Path("automation_summary.txt")

with open(summary_path, "w") as f:

    f.write("AUTOMATION SUMMARY\n")
    f.write("=" * 80 + "\n\n")

    f.write(f"Successful Experiments: {len(successful)}\n")
    f.write(f"Failed Experiments    : {len(failed)}\n\n")

    f.write("SUCCESSFUL:\n")
    for s in successful:
        f.write(f"  - {s}\n")

    f.write("\nFAILED:\n")
    for fail in failed:
        f.write(f"  - {fail}\n")

print(f"\nSaved summary → {summary_path.resolve()}")
