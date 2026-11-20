# Traffic-light perception

`avlite_signal_perception` classifies RGB crops and combines associated signal
heads. It depends on NumPy and OpenCV and can be used without AVLite or CARLA.
The AVLite detector supplies frames, projected head geometry and timestamps.

## Install

From the plugin directory, with the AVLite environment active:

```bash
python install.py
python install.py --check
```

The installer installs `requirements.txt` and downloads or verifies the pinned
YOLO model for object detection. The traffic-light MLP weights are bundled.
Use `python install.py --autoware` to fetch the optional Autoware classifier.

For the classifier library alone, use `python -m pip install .`.
External weights are cached under `$XDG_CACHE_HOME/avlite_signal_perception`,
or `~/.cache/avlite_signal_perception` by default. Matching files in the legacy
`models/` directory can be reused. Downloads are atomic and SHA-256 checked.
Missing models are fetched at detector construction, never during a frame tick.
A failed download prevents detector startup.

## Configuration

| Setting | Value |
| --- | --- |
| `sensor_signal_backend` | `trained_rgb` for the bundled CARLA-adapted MLP; `autoware` for the experimental MobileNet |
| `sensor_signal_model_path` | Replacement weights; empty selects the default artifact |
| `sensor_signal_provider` | Name of an explicitly registered classifier implementation |

Stop execution and reload the stack after changing models. Reloading also
clears previous temporal confirmations.

The trained RGB backend uses surveyed signal-head proposals. Its NPZ contains
`w1[3072,96]`, `b1[96]`, `w2[96,4]` and `b2[4]`. Input is RGB resized to 32×32
and scaled by 1/255; outputs are Red, Yellow, Green and Unknown.

The Autoware backend uses YOLO proposals and expects normalized 224×224 input
with an 11-class probability output. Replacement weights must match the selected
architecture and preprocessing. See the model metadata and
[SENSOR_VALIDATION.md](../validation/perception.md) for provenance and measured errors.
Neither backend has been validated for deployment on a real vehicle.

## Custom classifier

A classifier accepts an RGB crop and returns `SignalState`. Register its
constructor before creating the AVLite detector:

```python
from avlite_signal_perception import SignalState
from avlite_signal_perception.provider import register_classifier


class MyClassifier:
    def __init__(self, model=None):
        self.model_path = model

    def __call__(self, rgb):
        # Replace with inference; keep uncertain results Unknown.
        return SignalState(reason="not_implemented")


register_classifier("my_classifier", MyClassifier)
```

Set `sensor_signal_provider: my_classifier` to use it. The provider replaces RGB
classification; the backend still determines proposal and association policy.
`SignalState` carries color, original label, confidence and rejection reason.

The AVLite adapter checks frame age and requires three distinct green
observations. Surveyed-head fusion also requires two agreeing heads and rejects
conflicting visible colors. Head identity comes from calibrated static geometry.

## CARLA profile

```bash
python install.py --profile-output ~/.config/avlite/sensor-road-trained.yaml \
  --map-path /path/to/Town10HD_Opt.xodr \
  --bridge-path /path/to/avlite-bridge-carla
```

Use the matching Town10 map and a separately installed CARLA server, Python API
and bridge. The generator resolves plugin, route, survey and telemetry paths
for this checkout and refuses to overwrite an existing profile. Generate a new
profile after moving the checkout. The historical YAML files in `profiles/`
contain development-machine paths.

## Tests

The 10 November packaging check loaded the wheel in an isolated directory and
classified red, yellow and green fixtures without importing AVLite or CARLA.
The focused suite passed 188 tests with four deselected; a subsequent run passed
all 192. Live CARLA signal checks passed 16/16 cases. The executor check blocked
dynamic ground-truth APIs and verified `PERCEPTION_STOP` on a black frame.

The report is `logs/signal_package/validation_summary.json` in the development
workspace. These are packaging and integration checks for that revision.
Full-route results are in [SENSOR_VALIDATION.md](../validation/perception.md); current
community-package checks are in [testing.md](../testing.md).
