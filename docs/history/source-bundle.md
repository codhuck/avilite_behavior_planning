# Matching source bundle

> Historical matching-core distribution. For the standalone community release
> on official AVLite 0.6.7, follow [the current README](../../README.md) and
> [community compatibility](../community-plugin.md).

The plugin requires local changes in AVLite 0.6.6; CARLA scenarios also require
the matching CARLA bridge. The
`avlite-planning` source bundle includes these matching sources, maps, plugin
profiles and regression tests. It is not an upstream AVLite release or a claim
of compatibility with an arbitrary installed AVLite version.

From the extracted `avlite-planning` directory, in a Python 3.10+ environment,
start with the BasicSim coordinator check:

```bash
python -m pip install -e .
python tools/verify_planning_bundle.py
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
python avlite-behavior-hdmap/tools/coordinator_demo.py \
  --repeat 3 --output logs/coordinator-check.json
python avlite-behavior-hdmap/tools/prepare_coordinator_profile.py \
  --mode road --output "$PWD/logs/profiles/coordinator-road.yaml"
AVLITE_CONFIG_DIR="$PWD/logs/profiles" \
python avlite-behavior-hdmap/tools/check_coordinator_profile.py \
  --profile coordinator-road --output logs/profile-check.json
```

This path needs neither CARLA nor external detector weights. See
[task coordination](../task-coordination.md) for the race profile, graphical launch,
strategy catalog and sensor interface. The 18 November matching archive is
`avlite-planning-2026-11-18-coordinator.zip`. The 19 November follow-up archive,
`avlite-planning-2026-11-19-validated.zip`, adds the live CARLA checker and the
handover fixes described in [its validation record](../validation/coordination-carla-validation.md).

For the separate CARLA sensor demonstration:

```bash
python -m pip install -e .
python avlite-behavior-hdmap/install.py
python tools/verify_planning_bundle.py
python avlite-behavior-hdmap/install.py --check
python avlite-behavior-hdmap/install.py --behavior-tree \
  --profile-output ~/.config/avlite/sensor-road-tree.yaml \
  --map-path "$PWD/avlite/data/Town10HD_Opt.xodr" \
  --bridge-path "$PWD/avlite-bridge-carla"
```

The installer resolves paths from the extracted directory. It refuses to
replace an existing profile. Historical profiles inside the source tree are
examples; generate a new one instead of copying developer-machine paths.

The archive excludes user settings, logs, caches, Git history and external ONNX
weights. `install.py` downloads the pinned YOLO model with a checksum check;
the trained RGB classifier is bundled. Downloads need internet access. The
CARLA simulator and matching Python API are separate prerequisites (tested
version: 0.9.16). A graphical session is needed for the AVLite visualizer.

Start the CARLA server, run `python -m avlite`, select `sensor-road-tree`, reload
the stack and press Start. Do not run two clients advancing the same simulator.
The verifier performs an offline source-integrity/import/profile check; it does
not start CARLA or certify a complete driving run.

`SOURCE_MANIFEST.json` contains SHA-256 values for every bundled source file.
To make a new bundle from this workspace:

```bash
python tools/build_planning_bundle.py releases/avlite-planning.zip
```

The builder refuses to overwrite an existing archive. After unpacking into a
new directory, rerun the verifier and tests there to catch hidden dependencies
on the developer's checkout. See [requirements status](requirements-status.md)
for the boundaries of the available validation.
