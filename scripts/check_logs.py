# scripts/check_logs.py

from pathlib import Path
import pandas as pd

CLIENT_ROOT = Path("../Client_Experimentation_final")
SERVER_ROOT = Path("../Server_Experimentation_final")

OUTPUT_FILE = Path("log_columns_report.txt")


def write_line(text, file_handle):
    print(text)
    file_handle.write(text + "\n")


def print_csv_columns(csv_path, file_handle):
    try:
        df = pd.read_csv(csv_path, nrows=5)

        write_line("\n" + "=" * 100, file_handle)
        write_line(f"FILE: {csv_path}", file_handle)
        write_line("-" * 100, file_handle)
        write_line("Columns:", file_handle)

        for col in df.columns:
            write_line(f"  - {col}", file_handle)

        write_line(f"Rows sampled: {len(df)}", file_handle)

    except Exception as e:
        write_line("\n" + "=" * 100, file_handle)
        write_line(f"ERROR reading: {csv_path}", file_handle)
        write_line(f"Reason: {e}", file_handle)


def scan_directory(root_path, file_handle):
    write_line("\n" + "#" * 100, file_handle)
    write_line(f"SCANNING DIRECTORY: {root_path}", file_handle)
    write_line("#" * 100, file_handle)

    for csv_file in sorted(root_path.rglob("*.csv")):
        print_csv_columns(csv_file, file_handle)


if __name__ == "__main__":

    with open(OUTPUT_FILE, "w") as report_file:

        if not CLIENT_ROOT.exists():
            write_line(f"Client root not found: {CLIENT_ROOT}", report_file)

        if not SERVER_ROOT.exists():
            write_line(f"Server root not found: {SERVER_ROOT}", report_file)

        scan_directory(CLIENT_ROOT, report_file)
        scan_directory(SERVER_ROOT, report_file)

        write_line("\n\nDONE SCANNING ALL LOG FILES", report_file)

    print(f"\nReport saved to: {OUTPUT_FILE.resolve()}")