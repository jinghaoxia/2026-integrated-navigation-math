"""案例2 GNSS单历元伪距定位。

直接运行本脚本，结果默认保存到同级 outputs 目录。
"""
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

def pseudorange_factor(key, satellite, observation, sigma):
    """The Vector4 state is [ECEF X, ECEF Y, ECEF Z, c*clock_bias], in metres."""
    def error(this, values, jacobians):
        state = values.atVector(key)
        delta = state[:3] - satellite
        distance = np.linalg.norm(delta)
        if jacobians is not None:
            jacobians[0] = np.asfortranarray(np.r_[delta / distance, 1.0].reshape(1, 4))
        return np.array([distance + state[3] - observation])
    return gtsam.CustomFactor(gtsam.noiseModel.Isotropic.Sigma(1, sigma), [key], error)


def run(output_dir=None):
    """执行案例，保存图表和数据，并返回评价指标。"""
    OUT = Path(output_dir) if output_dir is not None else ROOT / "outputs"
    OUT.mkdir(parents=True, exist_ok=True)
    lat, lon = np.deg2rad([30.0, 114.0])
    a, e2 = 6378137.0, 6.69437999014e-3
    height = 50.0
    n = a / np.sqrt(1-e2*np.sin(lat)**2)
    receiver = np.array([(n+height)*np.cos(lat)*np.cos(lon),
                         (n+height)*np.cos(lat)*np.sin(lon),
                         (n*(1-e2)+height)*np.sin(lat)])
    # Columns are East, North, Up unit vectors in ECEF.
    enu_to_ecef = np.array([[-np.sin(lon), -np.sin(lat)*np.cos(lon), np.cos(lat)*np.cos(lon)],
                            [ np.cos(lon), -np.sin(lat)*np.sin(lon), np.cos(lat)*np.sin(lon)],
                            [0., np.cos(lat), np.sin(lat)]])
    az, el = np.deg2rad([0, 50, 100, 150, 200, 250, 300, 340]), np.deg2rad([25, 50, 35, 65, 20, 45, 30, 75])
    los_enu = np.c_[np.cos(el)*np.sin(az), np.cos(el)*np.cos(az), np.sin(el)]
    satellites = receiver + ((20.2e6 + np.arange(8)*1e5)[:,None] * los_enu) @ enu_to_ecef.T
    sigma, true_bias = 3.0, 75.0
    noise = np.random.default_rng(7).normal(0, sigma, 8)
    observations = np.linalg.norm(receiver - satellites, axis=1) + true_bias + noise
    truth = np.r_[receiver, true_bias]
    start = truth + np.array([100., -80., 60., -75.])
    key = gtsam.symbol('x', 0)
    graph, initial = gtsam.NonlinearFactorGraph(), gtsam.Values()
    initial.insert(key, start)
    for sat, rho in zip(satellites, observations):
        graph.add(pseudorange_factor(key, sat, rho, sigma))
    # Validate the analytic Jacobian independently with central differences.
    delta = start[:3] - satellites[0]
    analytic = np.r_[delta/np.linalg.norm(delta), 1.]
    numeric = []
    for k in range(4):
        step = np.zeros(4); step[k] = .25
        def h(x): return np.linalg.norm(x[:3]-satellites[0]) + x[3]
        numeric.append((h(start+step)-h(start-step))/(.5))
    assert np.max(np.abs(analytic-numeric)) < 1e-6
    params = gtsam.LevenbergMarquardtParams()
    params.setMaxIterations(50)
    params.setRelativeErrorTol(1e-9)
    opt = gtsam.LevenbergMarquardtOptimizer(graph, initial, params)
    result = opt.optimize()
    estimate = result.atVector(key)
    residual0 = np.linalg.norm(start[:3]-satellites, axis=1)+start[3]-observations
    residual1 = np.linalg.norm(estimate[:3]-satellites, axis=1)+estimate[3]-observations
    los = (receiver-satellites)/np.linalg.norm(receiver-satellites, axis=1)[:,None]
    design = np.c_[los, np.ones(8)]
    assert np.linalg.matrix_rank(design) == 4
    assert graph.error(result) < graph.error(initial)
    cov = gtsam.Marginals(graph, result).marginalCovariance(key)
    enu_error = enu_to_ecef.T @ (estimate[:3]-receiver)
    fig, axs = plt.subplots(1,2,figsize=(9,3.5))
    axs[0].plot(range(1,9),residual0,'o-',color='#C17735',label='Initial')
    axs[0].plot(range(1,9),residual1,'s-',color='#236D9D',label='Optimized')
    axs[0].axhline(0,color='.5',lw=.7)
    axs[0].set(xlabel='Satellite index',ylabel='Pseudorange residual (m)')
    axs[0].legend(); axs[0].grid(alpha=.2)
    axs[1].bar(['East','North','Up'],enu_error,color='#236D9D',width=.55)
    axs[1].axhline(0,color='.5',lw=.7)
    axs[1].set(ylabel='ENU position error (m)')
    axs[1].grid(axis='y',alpha=.2)
    fig.tight_layout()
    fig.savefig(OUT/'gnss_results.png',dpi=220)
    fig.savefig(OUT/'gnss_results.svg'); plt.close(fig)
    fig, ax = plt.subplots(figsize=(4.2,3))
    ax.plot(range(1,9),residual0,'o-',color='#C17735',label='Initial')
    ax.plot(range(1,9),residual1,'s-',color='#236D9D',label='Optimized')
    ax.set(xlabel='Satellite index',ylabel='Residual/m',xticks=[1,2,4,6,8])
    ax.axhline(0,color='.5',lw=.7); ax.legend(); ax.grid(alpha=.2); ax.tick_params(direction='in')
    fig.tight_layout(); fig.savefig(OUT/'gnss_residual.png',dpi=300); fig.savefig(OUT/'gnss_residual.svg'); plt.close(fig)
    with (OUT/'gnss_observations.csv').open('w',newline='',encoding='utf-8') as f:
        w=csv.writer(f);w.writerow(['satellite','ecef_x_m','ecef_y_m','ecef_z_m','corrected_pseudorange_m','sigma_m'])
        for i,(sat,rho) in enumerate(zip(satellites,observations)):w.writerow([i+1,*sat,rho,sigma])
    return dict(satellites=8, noise_sigma_m=sigma, seed=7,
                truth=truth.tolist(), initial=start.tolist(), estimate=estimate.tolist(),
                initial_objective=float(graph.error(initial)), final_objective=float(graph.error(result)),
                initial_position_error_m=float(np.linalg.norm(start[:3]-receiver)),
                final_position_error_m=float(np.linalg.norm(estimate[:3]-receiver)),
                initial_residual_rms_m=float(np.sqrt(np.mean(residual0**2))),
                final_residual_rms_m=float(np.sqrt(np.mean(residual1**2))),
                clock_bias_ns=float(estimate[3]/299792458.*1e9),
                enu_error_m=enu_error.tolist(), covariance=cov.tolist(),
                gdop=float(np.sqrt(np.trace(np.linalg.inv(design.T@design)))),
                iterations=opt.iterations(), jacobian_max_abs_error=float(np.max(np.abs(analytic-numeric))))

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
