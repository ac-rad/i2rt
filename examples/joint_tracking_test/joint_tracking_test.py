"""Joint-space sinusoid tracking test for a single YAM-family arm.

Moves the arm to a nominal home pose, then superimposes a small, smoothly ramped sinusoid on each
joint in turn (or all at once), streaming position + velocity-feedforward commands while recording
the commanded and measured joint state. Saves everything to an .npz file, prints a per-joint
summary (RMS / max error, amplitude ratio, phase lag) and plots the tracking error.

Typical use:

    uv run python examples/joint_tracking_test/joint_tracking_test.py --can-channel can0

Re-plot an earlier recording without touching the robot:

    uv run python examples/joint_tracking_test/joint_tracking_test.py \
        --plot-only tracking_20261116_120000.npz
"""

import json
import logging
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Literal, Optional, Tuple

import numpy as np
import tyro

from i2rt.robots.get_robot import get_yam_robot
from i2rt.robots.utils import ArmType, GripperType

ARM_DOF = 6


@dataclass
class Args:
    arm: Literal["yam", "yam_pro", "yam_ultra", "yam_ultra_2", "big_yam"] = "yam_ultra"
    """Arm variant. yam_ultra is the v1 Ultra; use yam_ultra_2 for the v2 hardware."""
    gripper: Literal["crank_4310", "linear_3507", "linear_4310", "flexible_4310", "no_gripper"] = "linear_4310"
    """Gripper mounted on the arm. It is held at its starting position for the whole test."""
    can_channel: str = "can0"
    sim: bool = False
    """Run against the MuJoCo SimRobot (which teleports to commands, so error is ~0: a plumbing check only)."""

    home: Tuple[float, float, float, float, float, float] = (0.0, 1.0, 1.0, 0.0, 0.0, 0.0)
    """Nominal home pose (rad). The default puts the tool ~0.38 m forward and ~0.40 m up."""
    amplitude: Tuple[float, ...] = (0.1,)
    """Sinusoid amplitude (rad): one value for all joints, or six values, one per joint."""
    freq: float = 0.5
    """Sinusoid frequency (Hz)."""
    cycles: int = 6
    """Cycles per joint. The first and last cycle are a smooth amplitude ramp, so >= 3."""
    joints: Tuple[int, ...] = (0, 1, 2, 3, 4, 5)
    """Joints to excite (0-indexed)."""
    mode: Literal["sequential", "simultaneous"] = "sequential"
    """sequential: one joint at a time (errors attributable to that joint and its coupling).
    simultaneous: all selected joints at once."""
    vel_ff: bool = True
    """Send the analytic desired velocity alongside the position target (MIT-mode velocity term)."""
    settle_s: float = 1.5
    """Hold time at home before the first and after every excitation segment."""
    rate_hz: float = 200.0
    """Command / logging rate. The robot's own control loop runs independently at ~1 kHz."""
    max_speed: float = 0.3
    """Max joint speed (rad/s) for the moves to home and back to the start pose."""
    abort_error: float = 0.3
    """Abort the test if any joint's tracking error exceeds this (rad)."""
    return_to_start: bool = True
    """At the end, move back to the pose the arm started in before releasing the motors."""
    out_dir: str = "."
    """Directory for the .npz recording and the .png plot."""
    plot: bool = True
    """Plot after the run."""
    show: bool = True
    """Open an interactive plot window (the PNG is saved either way)."""
    plot_only: Optional[str] = None
    """Skip the robot entirely and plot this existing .npz recording."""


# ---------------------------------------------------------------------------
# Trajectory
# ---------------------------------------------------------------------------


@dataclass
class Segment:
    t_start: float
    duration: float
    joints: Tuple[int, ...]  # empty -> hold at home


def build_schedule(args: Args) -> Tuple[List[Segment], float]:
    seg_T = args.cycles / args.freq
    segments: List[Segment] = []
    t = args.settle_s
    groups = [(j,) for j in args.joints] if args.mode == "sequential" else [tuple(args.joints)]
    for group in groups:
        segments.append(Segment(t, seg_T, group))
        t += seg_T + args.settle_s
    return segments, t


def envelope(tau: float, T: float, ramp: float) -> Tuple[float, float]:
    """Raised-cosine ramp up over ``ramp`` s, flat, ramp down over the last ``ramp`` s. Returns (e, de/dt)."""
    if tau <= 0.0 or tau >= T:
        return 0.0, 0.0
    if tau < ramp:
        x = np.pi * tau / ramp
        return 0.5 * (1 - np.cos(x)), 0.5 * np.pi / ramp * np.sin(x)
    if tau > T - ramp:
        x = np.pi * (T - tau) / ramp
        return 0.5 * (1 - np.cos(x)), -0.5 * np.pi / ramp * np.sin(x)
    return 1.0, 0.0


def desired(
    t: float, home: np.ndarray, amp: np.ndarray, args: Args, segments: List[Segment]
) -> Tuple[np.ndarray, np.ndarray, int]:
    """Desired arm (q, qd) at time t, and the index of the active segment (-1 = holding)."""
    q = home.copy()
    qd = np.zeros(ARM_DOF)
    w = 2 * np.pi * args.freq
    ramp = 1.0 / args.freq
    for i, seg in enumerate(segments):
        tau = t - seg.t_start
        if 0.0 <= tau <= seg.duration:
            e, de = envelope(tau, seg.duration, ramp)
            s, c = np.sin(w * tau), np.cos(w * tau)
            for j in seg.joints:
                q[j] += amp[j] * e * s
                qd[j] = amp[j] * (de * s + e * w * c)
            return q, qd, i
    return q, qd, -1


def min_jerk_move(robot: Any, q_from: np.ndarray, q_to: np.ndarray, extra: np.ndarray, args: Args) -> None:
    """Min-jerk joint move of the arm from q_from to q_to, holding the extra (gripper) DOFs."""
    dist = float(np.max(np.abs(q_to - q_from)))
    # Peak min-jerk speed is 1.875 * dist / T.
    T = max(2.0, 1.875 * dist / args.max_speed)
    logging.info(f"Moving {dist:.3f} rad max-joint over {T:.1f} s")
    dt = 1.0 / args.rate_hz
    t0 = time.perf_counter()
    while True:
        s = min((time.perf_counter() - t0) / T, 1.0)
        a = 10 * s**3 - 15 * s**4 + 6 * s**5
        robot.command_joint_pos(np.concatenate([q_from + a * (q_to - q_from), extra]))
        if s >= 1.0:
            break
        time.sleep(dt)


# ---------------------------------------------------------------------------
# Run
# ---------------------------------------------------------------------------


def check_limits(home: np.ndarray, amp: np.ndarray, joint_limits: Optional[np.ndarray], margin: float = 0.05) -> None:
    if joint_limits is None:
        logging.warning("Robot reported no joint limits; skipping the limit check")
        return
    lo, hi = joint_limits[:, 0] + margin, joint_limits[:, 1] - margin
    bad = [j for j in range(ARM_DOF) if home[j] - amp[j] < lo[j] or home[j] + amp[j] > hi[j]]
    if bad:
        rows = ", ".join(
            f"joint{j + 1}: [{home[j] - amp[j]:.3f}, {home[j] + amp[j]:.3f}] vs limits [{lo[j]:.3f}, {hi[j]:.3f}]"
            for j in bad
        )
        raise ValueError(f"home +/- amplitude leaves the joint limits (with {margin} rad margin): {rows}")


def run(args: Args) -> Path:
    if args.cycles < 3:
        raise ValueError("--cycles must be >= 3 (one ramp-up, >= 1 steady, one ramp-down)")
    if any(j < 0 or j >= ARM_DOF for j in args.joints):
        raise ValueError(f"--joints must be in 0..{ARM_DOF - 1}")
    amp = np.full(ARM_DOF, args.amplitude[0]) if len(args.amplitude) == 1 else np.asarray(args.amplitude, float)
    if amp.shape != (ARM_DOF,):
        raise ValueError("--amplitude takes 1 or 6 values")
    home = np.asarray(args.home, float)
    segments, t_end = build_schedule(args)
    logging.info(f"Test schedule: {len(segments)} segment(s), {t_end:.1f} s of excitation + holds")

    robot = get_yam_robot(
        channel=args.can_channel,
        arm_type=ArmType.from_string_name(args.arm),
        gripper_type=GripperType.from_string_name(args.gripper),
        sim=args.sim,
    )
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"tracking_{datetime.now().strftime('%Y%m%d_%H%M%S')}.npz"

    log: Dict[str, List[Any]] = {k: [] for k in ("t", "q_des", "qd_des", "q", "qd", "eff", "segment")}
    info = robot.get_robot_info()
    status = "incomplete"
    try:
        check_limits(home, amp, info.get("joint_limits"), margin=0.05)
        n_dofs = robot.num_dofs()
        obs = robot.get_observations()
        q_start = np.asarray(obs["joint_pos"][:ARM_DOF], float).copy()
        extra = np.asarray(obs["gripper_pos"], float).copy() if n_dofs > ARM_DOF else np.zeros(0)
        logging.info(f"Start pose: {np.round(q_start, 3)}  -> home: {np.round(home, 3)}")

        min_jerk_move(robot, q_start, home, extra, args)

        logging.info("At home. Starting excitation (Ctrl+C to stop early and return to start).")
        dt = 1.0 / args.rate_hz
        zeros_extra = np.zeros_like(extra)
        t0 = time.perf_counter()
        next_tick = t0
        last_seg = -2
        try:
            while True:
                t = time.perf_counter() - t0
                if t > t_end:
                    break
                # Measure first, then evaluate the reference at the same instant: the error then
                # includes the full command -> motion latency, which is what we want to see.
                obs = robot.get_observations()
                q_des, qd_des, seg = desired(t, home, amp, args, segments)
                q = np.asarray(obs["joint_pos"][:ARM_DOF], float)
                if np.max(np.abs(q_des - q)) > args.abort_error:
                    j = int(np.argmax(np.abs(q_des - q)))
                    raise RuntimeError(
                        f"joint{j + 1} tracking error {q_des[j] - q[j]:.3f} rad exceeds --abort-error {args.abort_error}"
                    )
                if args.vel_ff:
                    robot.command_joint_state(
                        {"pos": np.concatenate([q_des, extra]), "vel": np.concatenate([qd_des, zeros_extra])}
                    )
                else:
                    robot.command_joint_pos(np.concatenate([q_des, extra]))

                log["t"].append(t)
                log["q_des"].append(q_des)
                log["qd_des"].append(qd_des)
                log["q"].append(q.copy())
                log["qd"].append(np.asarray(obs["joint_vel"][:ARM_DOF], float).copy())
                log["eff"].append(np.asarray(obs["joint_eff"][:ARM_DOF], float).copy())
                log["segment"].append(seg)

                if seg != last_seg and seg >= 0:
                    names = ", ".join(f"joint{j + 1}" for j in segments[seg].joints)
                    logging.info(f"Exciting {names}")
                last_seg = seg

                next_tick += dt
                sleep = next_tick - time.perf_counter()
                if sleep > 0:
                    time.sleep(sleep)
                else:
                    next_tick = time.perf_counter()  # fell behind; don't try to catch up
            status = "complete"
        except KeyboardInterrupt:
            status = "interrupted"
            logging.warning("Interrupted: stopping excitation")
        except RuntimeError as e:
            status = "aborted"
            logging.error(f"Aborting: {e}")

        # Hold where we are, then finish the way we came.
        q_now = np.asarray(robot.get_observations()["joint_pos"][:ARM_DOF], float).copy()
        if args.return_to_start:
            min_jerk_move(robot, q_now, q_start, extra, args)
    finally:
        if log["t"]:
            np.savez(
                out_path,
                **{k: np.asarray(v) for k, v in log.items()},
                home=home,
                amplitude=amp,
                segment_joints=json.dumps([list(s.joints) for s in segments]),
                args=json.dumps(asdict(args)),
                kp=np.asarray(info.get("kp", [])),
                kd=np.asarray(info.get("kd", [])),
                status=status,
            )
            logging.info(f"Saved {len(log['t'])} samples ({status}) to {out_path}")
        robot.close()
    return out_path


# ---------------------------------------------------------------------------
# Analysis + plotting
# ---------------------------------------------------------------------------


def fit_sinusoid(t: np.ndarray, y: np.ndarray, freq: float) -> Tuple[float, float]:
    """Least-squares fit y ~ a sin(wt) + b cos(wt) + c. Returns (amplitude, phase rad)."""
    w = 2 * np.pi * freq
    A = np.column_stack([np.sin(w * t), np.cos(w * t), np.ones_like(t)])
    (a, b, _c), *_ = np.linalg.lstsq(A, y, rcond=None)
    return float(np.hypot(a, b)), float(np.arctan2(b, a))


def summarize(data: Dict[str, Any]) -> List[Dict[str, Any]]:
    t, q_des, q, seg = data["t"], data["q_des"], data["q"], data["segment"]
    args = json.loads(str(data["args"]))
    seg_joints = json.loads(str(data["segment_joints"]))
    freq, cycles = args["freq"], args["cycles"]
    err = q_des - q
    rows = []
    for i, joints in enumerate(seg_joints):
        mask = seg == i
        if not mask.any():
            continue
        tau = t[mask] - t[mask][0]
        T = cycles / freq
        steady = (tau > 1.0 / freq) & (tau < T - 1.0 / freq)  # exclude the ramps
        for j in joints:
            a_des, p_des = fit_sinusoid(tau[steady], q_des[mask][steady, j], freq)
            a_meas, p_meas = fit_sinusoid(tau[steady], q[mask][steady, j], freq)
            lag = (p_des - p_meas + np.pi) % (2 * np.pi) - np.pi
            others = [k for k in range(ARM_DOF) if k not in joints]
            rows.append(
                {
                    "joint": j + 1,
                    "rms_mrad": 1e3 * float(np.sqrt(np.mean(err[mask][:, j] ** 2))),
                    "max_mrad": 1e3 * float(np.max(np.abs(err[mask][:, j]))),
                    "gain": a_meas / a_des if a_des > 0 else float("nan"),
                    "lag_ms": 1e3 * lag / (2 * np.pi * freq),
                    "coupling_rms_mrad": 1e3 * float(np.sqrt(np.mean(err[mask][:, others] ** 2))) if others else 0.0,
                }
            )
    return rows


def plot(npz_path: Path, show: bool) -> None:
    data = dict(np.load(npz_path, allow_pickle=False))
    rows = summarize(data)
    print(f"\nTracking summary ({npz_path.name}, status={data.get('status', '?')}):")
    print(f"{'joint':>5} {'rms err':>10} {'max err':>10} {'gain':>7} {'lag':>8} {'other-joint rms':>16}")
    for r in rows:
        print(
            f"{r['joint']:>5} {r['rms_mrad']:>7.2f} mrad {r['max_mrad']:>7.2f} mrad {r['gain']:>7.3f} "
            f"{r['lag_ms']:>5.1f} ms {r['coupling_rms_mrad']:>11.2f} mrad"
        )

    try:
        import matplotlib

        if not show:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print(f"\nmatplotlib is not installed; run `uv sync --locked` and re-plot with --plot-only {npz_path}")
        return

    t, q_des, q, eff, seg = data["t"], data["q_des"], data["q"], data["eff"], data["segment"]
    seg_joints = json.loads(str(data["segment_joints"]))
    err_mrad = 1e3 * (q_des - q)

    fig, axes = plt.subplots(ARM_DOF, 3, sharex=True, figsize=(16, 13))
    for j in range(ARM_DOF):
        ax_pos, ax_err, ax_eff = axes[j]
        # Shade the windows where this joint is being excited.
        for i, joints in enumerate(seg_joints):
            if j in joints and (seg == i).any():
                ts = t[seg == i]
                for ax in axes[j]:
                    ax.axvspan(ts[0], ts[-1], color="tab:orange", alpha=0.08, lw=0)
        ax_pos.plot(t, q_des[:, j], "--", lw=1, label="desired")
        ax_pos.plot(t, q[:, j], lw=1, label="measured")
        ax_pos.set_ylabel(f"joint{j + 1}\n[rad]")
        ax_err.plot(t, err_mrad[:, j], lw=1, color="tab:red")
        ax_err.axhline(0, color="k", lw=0.5)
        ax_err.set_ylabel("error [mrad]")
        ax_eff.plot(t, eff[:, j], lw=1, color="tab:green")
        ax_eff.set_ylabel("effort [Nm]")
        for ax in axes[j]:
            ax.grid(True, alpha=0.3)
    axes[0, 0].legend(loc="upper right", fontsize=8)
    axes[0, 0].set_title("position")
    axes[0, 1].set_title("tracking error (desired - measured)")
    axes[0, 2].set_title("measured effort")
    for ax in axes[-1]:
        ax.set_xlabel("time [s]")
    args = json.loads(str(data["args"]))
    fig.suptitle(
        f"{args['arm']} sinusoid tracking: {args['freq']} Hz, amplitude {np.round(data['amplitude'], 3).tolist()} rad, "
        f"vel_ff={args['vel_ff']}, {args['mode']}"
    )
    fig.tight_layout()
    png = npz_path.with_suffix(".png")
    fig.savefig(png, dpi=120)
    print(f"\nSaved plot to {png}")
    if show:
        plt.show()


def main(args: Args) -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    if args.plot_only:
        plot(Path(args.plot_only), args.show)
        return
    out_path = run(args)
    if args.plot and out_path.exists():
        plot(out_path, args.show)


if __name__ == "__main__":
    main(tyro.cli(Args))
