# Traffic-light perception package

`avlite_signal_perception` is an independent Python package. It imports neither
AVLite nor CARLA and does not control a vehicle. It classifies RGB image crops
and combines explicitly associated signal heads. The AVLite adapter supplies
camera frames, static-map projections and timestamps, then writes the result
to `PerceptionModel.regulatory_observation`. The behavior planner reads that
observation; it does not run the classifier.

## Install

With the AVLite environment activated, from the cloned plugin directory:

```bash
python install.py
python install.py --check
```

The first command installs `requirements.txt` and verifies/downloads the pinned
YOLO artifact needed for people/vehicle detection. The trained traffic-light
weights are bundled. The usual AVLite clone + `pip install -r requirements.txt`
workflow also works: missing external models download on first detector creation,
not at import or during frame processing. Failed downloads/checksums raise a
startup error; they never enable a ground-truth fallback. To prefetch the optional
Autoware model, use `python install.py --autoware`.

The independent library can be installed with `pip install .` or built into a
wheel. It needs NumPy and OpenCV only. AVLite itself and the CARLA bridge/server
are separate application dependencies. No compiler, BehaviorTree.CPP or C++
wrapper is required by this implementation.

External weights live under `$XDG_CACHE_HOME/avlite_signal_perception` (default
`~/.cache/avlite_signal_perception`). A development checkout can reuse matching
files in its legacy `models/` directory. Downloads are atomic and SHA-256 checked.
No models are downloaded simply by importing the package.

## Choose weights

In the AVLite plugin settings, choose:

| Setting | Purpose |
| --- | --- |
| `sensor_signal_backend: trained_rgb` | Bundled CARLA-adapted model; surveyed head proposals |
| `sensor_signal_backend: autoware` | Experimental real-image-trained MobileNet; YOLO proposals |
| `sensor_signal_model_path` | Optional replacement weights with the selected architecture |
| `sensor_signal_provider` | Optional explicitly registered classifier implementation |

An empty path selects the default artifact. The MLP accepts an NPZ containing
`w1[3072,96]`, `b1[96]`, `w2[96,4]`, `b2[4]`. Classes are Red, Yellow, Green,
Unknown; input is resized RGB 32x32 scaled by 1/255. The Autoware adapter expects
its documented 224x224 normalized input and 11-class probability output. An
arbitrary ONNX model is not interchangeable just because its extension matches.
Stop execution and reload the stack after changing models. That also discards
old temporal confirmations.

Neither backend name means a model is qualified for real-road use. The bundled
model card lists training provenance and limitations. The independent test
results and individual-crop errors are documented in [SENSOR_VALIDATION.md](SENSOR_VALIDATION.md).

## Replace the classifier

```python
from avlite_signal_perception.provider import register_classifier, SignalPerception
from avlite_signal_perception import SignalState


class MyClassifier:
    def __init__(self, model=None):
        self.model_path = model

    def __call__(self, rgb):
        # Implement inference and uncertainty rejection here.
        return SignalState(reason="not_implemented")


register_classifier("my_classifier", MyClassifier)
service = SignalPerception(
    backend="trained_rgb", provider="my_classifier", model="/path/to/weights"
)
states, observations = service.recognize(rgb, detections=[], heads=projected_heads)
```

Register the provider explicitly in extension code before constructing the
adapter. Set `sensor_signal_provider: my_classifier` to use it in AVLite. The
backend controls proposal/association policy; the provider replaces RGB
classification. This is not dynamic execution of code embedded in a weights file.

`SignalState` contains color, original label, confidence and rejection reason.
Unknown must remain Unknown. The adapter owns fresh-frame checks and three-frame
green confirmation. Surveyed-head fusion requires two distinct agreeing heads
for green and rejects conflicting visible colours. Caller-supplied projected
head identity must come from calibrated static geometry, not nearest green.

## Portable demo profile

The repository's historical YAML files contain development-machine paths. For a
fresh checkout generate a new profile with actual local paths instead:

```bash
python install.py --profile-output ~/.config/avlite/sensor-road-trained.yaml \
  --map-path /path/to/Town10HD_Opt.xodr \
  --bridge-path /path/to/avlite-bridge-carla
```

This bundles the short demo route with the plugin and resolves plugin, geometry,
route and telemetry paths on the destination machine. It refuses to overwrite an
existing profile. This is a Town10 demo configuration, not a converter for an
arbitrary HD map. The matching map and separately installed CARLA bridge/server
are required. A standalone perception wheel needs none of those assets.

## Packaging regression checks (2026-11-10)

The independent wheel was installed into an isolated directory and classified
red, yellow and green fixtures without importing AVLite or CARLA. The focused
suite passed 188 tests (4 deselected). After the extraction, live CARLA signal
checks passed 16/16 cases. The AVLite executor check passed with dynamic ground
truth APIs blocked; a black camera frame produced `PERCEPTION_STOP`.

Evidence: `logs/signal_package/validation_summary.json` in the parent workspace.
These checks cover this extraction and packaging change, not a new full-route
drive or real-road validation. Earlier full-route evidence is documented in
`SENSOR_VALIDATION.md`.

The subsequent style-cleanup regression run passed all 192 tests with the default
marker filter disabled, including the four timing-sensitive tests.
