# Minimum Gello (Teleoperation)

Minimal leader-follower teleoperation using a YAM arm with a teaching handle (gello) for gripper control. Supports different arm and gripper assemblies, simulation mode, and local/remote visualization.

## Hardware Setup

- **Follower arm** — YAM arm with gripper (e.g. `linear_4310`) on `can1`
- **Leader arm** — YAM arm with teaching handle (`yam_teaching_handle_v1`) on `can0`

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
    --gripper yam_teaching_handle_v1 --mode leader --can-channel can0
```

The follower will slow-move to match the leader's starting position, then teleoperation begins immediately — no button press required. Squeeze the teaching handle to control the follower gripper. Pressing **button 0** on the handle toggles synchronization off/on.

> **Note:** Leader mode requires real hardware (`--sim` is not supported).

## Simulation and Other Assemblies

```bash
# Follower mode in simulation (no hardware required)
python examples/minimum_gello/minimum_gello.py --sim

# Different arm and gripper combinations
python examples/minimum_gello/minimum_gello.py --arm big_yam --gripper linear_4310 --sim
python examples/minimum_gello/minimum_gello.py --arm yam_pro --gripper flexible_4310 --sim
```

## Options

| Flag | Default | Description |
|------|---------|-------------|
| `--arm` | `yam` | Arm type (`yam`, `yam_pro`, `yam_ultra`, `yam_ultra_2`, `big_yam`, `no_arm`) |
| `--gripper` | `yam_teaching_handle_v1` | Gripper type (`crank_4310`, `linear_3507`, `linear_4310`, `flexible_4310`, `yam_teaching_handle_v1`, `yam_teaching_handle_v2`, `no_gripper`) |
| `--mode` | `follower` | Operation mode (`follower`, `leader`, `visualizer_local`, `visualizer_remote`) |
| `--sim` | `False` | Use SimRobot instead of real hardware |
| `--can-channel` | `can0` | CAN interface name |
| `--server-host` | `localhost` | Portal server host (for leader/remote visualizer) |
| `--server-port` | `11333` | Portal server port |
| `--bilateral-kp` | `0.0` | Bilateral force feedback gain (leader mode); `0` = leader in gravity-comp only |
| `--ee-mass` | XML default | Override the end-effector body's mass (kg) for gravity compensation. See [Overriding Handle Weight](#overriding-handle-weight) |

## Visualizer

To visualize the follower arm live in MuJoCo (while follower is running):

```bash
python examples/minimum_gello/minimum_gello.py \
    --gripper linear_4310 --mode visualizer_local --can-channel can1
```

## Overriding Handle Weight

Both shipped teaching-handle models carry the mass their ONShape export reports, and those
exports have real materials assigned. `yam_teaching_handle_v1` is 0.252595 kg of handle shell plus
0.0049991 kg of trigger, so 0.2576 kg hangs off the flange; `yam_teaching_handle_v2` is
0.238592 kg plus 0.00499911 kg, so 0.2436 kg. Gravity compensation already sees whichever you
select, and a stock handle needs no override.

`--ee-mass` is for handles that differ — older 3D-printed shells in particular. Weigh yours and
pass it only if it disagrees with the default for the handle you selected:

```bash
python examples/minimum_gello/minimum_gello.py --ee-mass 0.31 --can-channel can0
```

The override replaces the mass of the composed end-effector body outright: it sets the handle
shell's mass only, leaving the trigger's 0.005 kg alone, and it does not rescale the inertia
tensor, which stays at the export's values.

## Notes

- **Joint limits**: The follower will stop tracking if the leader reaches a joint limit. Restart both processes to recover.
- **Motor lights**: Blinking red = standby (normal before script starts). Solid green = active. If lights go red during teleop, a motor has faulted — power-cycle the arm.
- **CAN channel is down**: If you see `fail to communicate with motor`, check `ip link show canX` — the interface may be DOWN. Bring it up with `sudo ip link set canX up type can bitrate 1000000`.
