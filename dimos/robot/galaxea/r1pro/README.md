# Galaxea R1 Pro

18-DOF upper body (torso 4 + arm 7 + arm 7) over ROS 2 / FastDDS, holonomic
chassis, head stereo + wrist cameras, chassis lidar, 2 IMUs.

## Robot-side setup

Everything in this section runs **on the robot's onboard computer**, over ssh,
not on your workstation. The paths below are the robot's, and are the same on
every R1 Pro.

The robot runs `galaxea-dimos`: Galaxea's own ROS 2 driver (firmware V2.3.0)
with the changes dimos needs. It is not part of dimos. It is stored in dimos
cloud as one tarball, `galaxea-dimos-v2.3.0-dimos.1.tar.zst`, upload id
`56a9468a113444c5afa792a9c7877cd1`. Install and start it:

```bash
dimos login   # once per robot
dimos data pull 56a9468a113444c5afa792a9c7877cd1 --dest ~/galaxea-dimos.tar.zst
tar -I zstd -xf ~/galaxea-dimos.tar.zst -C ~
~/galaxea-dimos/start.sh
```

The tree must end up at `~/galaxea-dimos`. `start.sh` runs `~/canfd.sh`, which
ships with the robot and brings up the CAN FD interfaces, then restarts the
driver and waits until it publishes. Restarting ends every tmux session on the
robot, so the script lists them and stops unless you pass `--yes`.
`~/galaxea-dimos/README-dimos.md` lists every change from stock and why, and
`dimos-changes.diff` holds the exact diff.

To restart the driver by hand:

```bash
bash ~/canfd.sh
cd ~/galaxea-dimos/install/startup_config/share/startup_config/script
./robot_startup.sh kill
./robot_startup.sh boot ../sessions.d/ATCStandard/R1PROBody.d/
```

## Environment

- `ROS_DOMAIN_ID=1` (new-gen V2.3.0), `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`.
- ROS 2 Humble ships `rclpy` built for CPython 3.10, so the venv must use the
  robot's system interpreter. `.python-version` says 3.12, so pass `--python`
  explicitly rather than relying on direnv:

  ```bash
  sudo apt-get install -y libturbojpeg   # pyturbojpeg needs the native lib
  uv sync --python /usr/bin/python3.10 --python-preference only-system \
          --no-default-groups --extra base --extra manipulation --extra cpu
  uv pip install python-socketio         # runtime dep of the viewer, only declared in the lint group
  ```

  `--all-extras` does not work on the robot's arm64 board: the `scene` extra
  needs `usd-core` (x86_64/macOS wheels only) and `mapping` needs
  `gtsam-extended` (arm64 Linux wheels start at cp311). `mapping` is also
  reachable through `unitree` and `all`, so excluding it by name is not enough.
- The planning model fetches the vendor URDF from the pinned upstream repo.

## Blueprints

```bash
dimos run r1pro-coordinator     # connection + coordinator + viewer
dimos run r1pro-teleop          # + chassis teleop from the viewer
dimos run r1pro-nav             # + click-to-drive nav (costmap + A*)
dimos run r1pro-manipulation    # + dual-arm planning (experimental)
dimos run r1pro-planar-preview   # planar-base planning preview with fake hardware
```
