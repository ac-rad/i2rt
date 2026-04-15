# Minimum Gello (Teleoperation)

Minimal leader-follower teleoperation using a YAM arm. Supports a teaching handle (gello) with gripper control, or a plain arm with no gripper.

## Hardware Setup

- **Follower arm** — YAM arm with gripper (e.g. `linear_4310`) on `can1`
- **Leader arm** — YAM arm with teaching handle (`yam_teaching_handle`) on `can0`

Make sure both CAN interfaces are up before starting:

```bash
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000
```

## Running Leader-Follower

Start the **follower first**, then the **leader**. The follower must be up and listening before the leader connects.

**Terminal 1 — Follower:**
```bash
python examples/minimum_gello/minimum_gello.py \
    --gripper linear_4310 --mode follower --can-channel can1
```

**Terminal 2 — Leader (with teaching handle + gripper control):**
```bash
python examples/minimum_gello/minimum_gello.py \
    --gripper yam_teaching_handle --mode leader --can-channel can0
```

The follower will slow-move to match the leader's starting position, then teleoperation begins immediately — no button press required. Squeeze the teaching handle to control the follower gripper.

## Options

| Flag | Default | Description |
|------|---------|-------------|
| `--gripper` | `yam_teaching_handle` | Gripper type: `crank_4310`, `linear_3507`, `linear_4310`, `yam_teaching_handle`, `no_gripper` |
| `--mode` | `follower` | Mode: `follower`, `leader`, `visualizer_local`, `visualizer_remote` |
| `--can-channel` | `can0` | CAN interface name |
| `--server-host` | `localhost` | Host of the follower portal server (leader mode) |
| `--server-port` | `11333` | Port of the follower portal server |
| `--ee-mass` | XML default | Override link_6 mass (kg) for gravity compensation |

## Visualizer

To visualize the follower arm live in MuJoCo (while follower is running):

```bash
python examples/minimum_gello/minimum_gello.py \
    --gripper linear_4310 --mode visualizer_local --can-channel can1
```

## Notes

- **Joint limits**: The follower will stop tracking if the leader reaches a joint limit. Restart both processes to recover.
- **Motor lights**: Blinking red = standby (normal before script starts). Solid green = active. If lights go red during teleop, a motor has faulted — power-cycle the arm.
- **CAN channel is down**: If you see `fail to communicate with motor`, check `ip link show canX` — the interface may be DOWN. Bring it up with `sudo ip link set canX up type can bitrate 1000000`.
