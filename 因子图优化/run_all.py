"""从因子图优化目录统一运行两个教学案例。"""
from pathlib import Path
import argparse
import subprocess
import sys

ROOT = Path(__file__).resolve().parent
CASES = {
    "sphere": ROOT / "案例1-球形位姿图优化" / "sphere_demo.py",
    "gnss": ROOT / "案例2-GNSS单历元伪距定位" / "gnss_demo.py",
}

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", choices=("all", "sphere", "gnss"), default="all")
    args = parser.parse_args()
    selected = CASES.values() if args.case == "all" else [CASES[args.case]]
    for script in selected:
        print(f"运行 {script.parent.name}", flush=True)
        subprocess.run([sys.executable, str(script)], check=True)
