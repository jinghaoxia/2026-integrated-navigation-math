"""案例1 球形位姿图优化。

直接运行本脚本，结果默认保存到同级 outputs 目录。
"""
import time
import hashlib
import shutil
import tempfile
from pathlib import Path
import argparse
import csv
import json

import numpy as np
import gtsam
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 11})


def run(output_dir=None):
    """执行案例，保存图表和数据，并返回评价指标。"""
    OUT = Path(output_dir) if output_dir is not None else ROOT / "outputs"
    OUT.mkdir(parents=True, exist_ok=True)
    data = ROOT / "data" / "sphere.g2o"
    # GTSAM's Windows C++ file reader uses narrow paths; stage Chinese paths.
    with tempfile.TemporaryDirectory(prefix="sphere_") as temporary:
        staged = Path(temporary) / "sphere.g2o"
        shutil.copyfile(data, staged)
        graph, initial = gtsam.readG2o(str(staged), True)
    keys = sorted(initial.keys())
    edge_count = graph.size()
    # A tight prior removes the common global SE(3) gauge freedom.
    anchor = gtsam.noiseModel.Diagonal.Sigmas(np.full(6, 1e-6))
    graph.add(gtsam.PriorFactorPose3(keys[0], initial.atPose3(keys[0]), anchor))
    params = gtsam.LevenbergMarquardtParams()
    params.setMaxIterations(100)
    params.setRelativeErrorTol(1e-6)
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial, params)
    history = [float(optimizer.error())]
    started = time.perf_counter()
    # The public iterate API permits recording the actual objective history.
    for _ in range(100):
        previous = history[-1]
        optimizer.iterate()
        current = float(optimizer.error())
        history.append(current)
        if previous - current <= 1e-6 * max(previous, 1.0):
            break
    result = optimizer.values()
    elapsed = time.perf_counter() - started
    xyz0 = np.array([initial.atPose3(k).translation() for k in keys])
    xyz1 = np.array([result.atPose3(k).translation() for k in keys])
    assert result.size() == initial.size()
    assert np.isfinite(history).all() and history[-1] < history[0]
    # The same bounds and view prevent misleading visual scale comparisons.
    pts = np.vstack([xyz0, xyz1])
    mid = (pts.max(0) + pts.min(0)) / 2
    half = np.max(np.ptp(pts, axis=0)) * 0.52
    fig = plt.figure(figsize=(10.6, 4.8))
    for idx, (xyz, title, color) in enumerate([
        (xyz0, "Initial pose graph", "#C17735"),
        (xyz1, "Optimized pose graph", "#236D9D"),
    ]):
        ax = fig.add_subplot(1, 2, idx + 1, projection="3d")
        ax.plot(*xyz.T, color=color, lw=0.55)
        ax.set_title(title, fontsize=16)
        ax.set_xlabel("X/m",fontsize=16);ax.set_ylabel("Y/m",fontsize=16);ax.set_zlabel("Z/m",fontsize=16)
        ax.tick_params(labelsize=15)
        ax.set_xlim(mid[0]-half, mid[0]+half)
        ax.set_ylim(mid[1]-half, mid[1]+half)
        ax.set_zlim(mid[2]-half, mid[2]+half)
        ax.set_box_aspect((1, 1, 1))
        ax.view_init(elev=24, azim=-55)
    fig.subplots_adjust(left=.01, right=.96, bottom=.04, top=.89, wspace=.03)
    fig.savefig(OUT / "sphere_comparison.png", dpi=220)
    plt.close(fig)
    for xyz, name, color in [(xyz0, "sphere_initial", "#C17735"), (xyz1, "sphere_result", "#236D9D")]:
        fig = plt.figure(figsize=(4.1, 3.8))
        ax = fig.add_subplot(111, projection="3d")
        ax.plot(*xyz.T, color=color, lw=.5)
        ax.set(xlabel="X/m", ylabel="Y/m", zlabel="Z/m")
        ax.set_xlim(mid[0]-half, mid[0]+half); ax.set_ylim(mid[1]-half, mid[1]+half); ax.set_zlim(mid[2]-half, mid[2]+half)
        ax.set_box_aspect((1,1,1)); ax.view_init(elev=24,azim=-55)
        fig.subplots_adjust(left=0,right=.92,bottom=.03,top=1)
        fig.savefig(OUT/(name+'.png'),dpi=300); fig.savefig(OUT/(name+'.svg')); plt.close(fig)
    fig, ax = plt.subplots(figsize=(4.2, 2.35))
    ax.semilogy(range(len(history)), history, "o-", color="#236D9D", ms=3)
    ax.set(xlabel="LM iteration", ylabel="Objective F (unitless)")
    ax.tick_params(direction='in')
    ax.grid(alpha=.25)
    fig.tight_layout()
    fig.savefig(OUT / "sphere_convergence.png", dpi=220)
    fig.savefig(OUT / "sphere_convergence.svg")
    plt.close(fig)
    with (OUT / "sphere_objective.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f); writer.writerow(["iteration", "objective"])
        writer.writerows(enumerate(history))
    with tempfile.TemporaryDirectory(prefix="sphere_") as temporary:
        staged = Path(temporary) / "result.g2o"
        gtsam.writeG2o(graph, result, str(staged))
        shutil.copyfile(staged, OUT / "sphere_optimized.g2o")
    return dict(poses=len(keys), relative_factors=edge_count, total_factors=graph.size(),
                initial_objective=history[0], final_objective=history[-1],
                iterations=len(history)-1, elapsed_seconds=elapsed,
                anchor_translation_shift_m=float(np.linalg.norm(xyz1[0]-xyz0[0])),
                dataset_sha256=hashlib.sha256(data.read_bytes()).hexdigest())

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs",
                        help="图表、CSV 和指标 JSON 的输出目录")
    args = parser.parse_args()
    output = args.output_dir.resolve()
    metrics = run(output)
    (output / "metrics.json").write_text(
        json.dumps(metrics, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(metrics, ensure_ascii=False, indent=2))
