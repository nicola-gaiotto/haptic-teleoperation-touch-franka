# How this repository is kept up to date

The repository grows with the project. These conventions keep it tidy.

## Where new work goes

| New work | Location |
|---|---|
| A new ROS 2 package (e.g. the Franka bridge) | `ros2/<package_name>/` |
| A script that is not a ROS node | `tools/<topic>/` |
| Documentation of a new deliverable | `docs/0N_<short_name>.md`, linked from the README |
| New calibration results | `calibration/` (overwrite the JSON files) |
| Recorded data used in the docs | `data/<topic>/` (small files only) |
| Photos, GIFs, videos | `docs/media/` |

## Routine after each work session

```bash
cd ~/geomagic_ws/src/haptic-teleoperation-touch-franka
git status                     # what changed
git add -A
git commit -m "Short description of what was done"
git push
```

Commit message examples: `Add stylus orientation from gimbal encoders`,
`Tune gravity gain to 0.8`, `Docs: add Franka bridge design`.

## At each milestone

1. Update the status table in `README.md`.
2. Add an entry at the top of `CHANGELOG.md`.
3. Optionally create a tag, visible on GitHub as a release:

```bash
git tag -a v0.3.0 -m "Stylus orientation"
git push --tags
```

## Experiments without breaking the working version

```bash
git checkout -b feature/franka-bridge      # new branch for the experiment
# ... work, commit ...
git checkout main                          # back to the working version
git merge feature/franka-bridge            # when the experiment works
```
