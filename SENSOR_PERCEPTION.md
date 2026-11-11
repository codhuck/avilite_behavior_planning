# RGB/LiDAR perception

For a new installation, follow the [README](README.md) and use the generated
`sensor-road-trained` profile. Its default classifier is the bundled RGB MLP;
see [Trained RGB backend](#trained-rgb-backend). The `sensor-road` setup below
documents the earlier aperture classifier for reproducibility. Commands using
`tools/` require the parent development workspace.

`SensorRegulatoryDetector` replaces dynamic CARLA object queries in the
`sensor-road` profile. It runs before planning in the ordinary perception
pipeline. `SensorTelemetryTask` records its output without overwriting it.

## Data boundary

- Inputs: RGB pixels, non-semantic LiDAR points, intrinsics/extrinsics,
  acquisition timestamps, stack ego pose and static HD-map geometry.
- YOLOv8n ONNX detects people, vehicles, bicycles, traffic lights and STOP signs.
  LiDAR points inside the central portion of a detection provide its position.
  When LiDAR misses a person, visible feet can be projected onto a local ground plane estimated from low LiDAR returns. This is marked `camera_ground_plane` in telemetry. Cropped feet or an unobservable ground plane remain unknown and prevent an empty-crossing decision.
- Three-aspect signal state is inferred from the lit aperture in the RGB image.
  The classifier handles CARLA's desaturated bright lenses; it is specific to
  vertical three-lamp heads, not a general traffic-signal recognition model.
- A static signal-head survey associates image regions with OpenDRIVE IDs.
  It contains geometry only. No light colours, pedestrian positions or vehicle
  poses are cached in this file. Runtime refuses its association if the map's
  signal-definition positions do not match the survey.
- LiDAR foreground inside a crossing blocks it even when the neural detector
  misses the object's class. Clearance requires low-return visibility across
  the sampled route-conflict area with a six-metre lateral approach margin and 0.5 s of continuous clear observations.
- Missing, repeated, misaligned, dark or invalid frames produce Unknown.
  Green requires three distinct observations. Invalid sensor health or stale observations command a stop even away from mapped crossings. No ground-truth fallback exists.
- The optional Kalman tracker estimates motion from successive detections.
  Ground-truth DETECTION and TRACKING are disabled in the profile.
  The city profile uses 2 m following and 2 m stopping buffers (in addition
  to body extents), replacing the generic 10 m / 5 m defaults at 3 m/s.

**Retained map/localization assumptions:** route, lane topology, crossing polygons,
STOP locations and signal identity remain static map inputs. The CARLA demo
still uses simulator ego localization. This is sensor-driven dynamic perception,
not a mapless or end-to-end autonomy system. STOP detections are recorded;
the existing conservative mapped STOP rule is retained.

## Earlier aperture-backend setup

```bash
conda activate avlite-carla
cd /path/to/avlite
python -m pip install -r avlite-behavior-hdmap/requirements-sensors.txt
python tools/install_sensor_model.py
python tools/prepare_sensor_profile.py
```

The default sensor profile uses the 150 m `sensor_demo` route. The longer
`sensor_city` route remains available for further experiments.

Profile: `avlite-behavior-hdmap/profiles/sensor-road.yaml`. Install a copy under
`~/.config/avlite/sensor-road.yaml`, then select `sensor-road` in AVLite.
Do not enable CarlaRegulatoryTask or world DETECTION/TRACKING in this profile.
Camera rendering must be enabled. Run only one simulator-ticking client.

For a new map, first create a static survey with `tools/export_signal_geometry.py`.
This offline tooling may read map object geometry; it never reads light state.
A deployment outside CARLA would require a surveyed equivalent and new validation.

## Reproducible validation

```bash
python -m pytest test/plugins/test_sensor_regulatory.py test/plugins/test_hdmap_behavior_plugin.py -q
python tools/check_sensor_perception.py --output logs/sensor_perception_lights
python tools/run_hdmap_scenarios.py --routes sensor_demo --repeats 2 \
  --max-sim-time 200 --sensor-perception --walking-pedestrian --pedestrians 3 \
  --dispatcher --output logs/sensor_demo_validation
```

The scenario harness uses actor information only to create controlled fixtures
and to audit violations independently. It passes only `SensorFrame` and
`PerceptionModel` to the detector. Telemetry includes `source: rgb_lidar`,
detection confidence, fused position, signal ROIs and crossing coverage.

## Limits

The COCO model is not fine-tuned for CARLA. Small or occluded people can be missed.
LiDAR fusion is approximate and does not estimate full 3D object shape. Crossing
visibility is sampled, not a proof that every point is observable. Uncertain
crossings can keep the car stopped; no timeout silently turns Unknown into Clear.
Signal classification needs further evaluation under night, rain, glare,
occlusion, different signal layouts and localization/calibration error.
No real-vehicle safety claim is made.

The installed executor is checked by `python tools/check_sensor_profile.py`: dynamic GT APIs throw if called, and a black camera frame must produce `PERCEPTION_STOP`.

Completed results and evidence are listed in [SENSOR_VALIDATION.md](SENSOR_VALIDATION.md).

## Portable traffic-light backend (experimental, 2026-11-10)

The earlier CARLA aperture heuristic is explicitly `sensor_signal_backend:
carla_aperture`. It remains the default for compatibility with the recorded
CARLA validation. Those results do **not** validate the new backend.

`sensor_signal_backend: autoware` uses YOLO image detections followed by the
Autoware v4.0 MobileNetV2 vehicular state classifier. `traffic_light_vision.py`
has no CARLA, map, or localization dependency. It consumes RGB uint8 images,
returns colour, raw class, confidence and rejection reason. No fixed vertical
lamp positions or CARLA brightness thresholds are used in this mode.

Preprocessing follows the upstream classifier: RGB, aspect-preserving 224x224
resize with black padding on the bottom/right, ImageNet byte-scale mean/std.
ONNX output is already normalized probabilities; no second softmax is applied.
Confidence >=0.85 and a top-two gap >=0.25 are provisional rejection thresholds,
not calibrated probabilities of safety. Arrow classes are retained in diagnostics
but yield Unknown until maneuver-specific semantics are implemented.

Association is separate: detected boxes must overlap a uniquely identified,
projected surveyed signal. Multiple candidate signal IDs or conflicting head
states yield Unknown. Three fresh frames remain necessary for Green. Without a
matching survey the camera still reports detections, but no route signal gets
permission. Static HD-map geometry is allowed; dynamic simulator light states
are never queried. The supplied survey and ego localization remain CARLA demo
inputs and must be replaced for a physical vehicle.

Install optional dependencies and models in the execution environment:

```bash
python -m pip install -r avlite-behavior-hdmap/requirements-sensors.txt
python tools/install_sensor_model.py
python tools/install_signal_model.py
```

The repository profile `profiles/sensor-road-portable.yaml` demonstrates the
backend in the existing CARLA bridge. It has not been installed as the startup
profile, and no closed-loop success is claimed for it.

Offline evaluation, without CARLA:

```bash
python tools/check_public_signal_samples.py
python tools/replay_signal_images.py /path/to/manifest.json --output logs/signals.json
```

Manifest: a JSON list of `{"image":"frame.jpg"}` for full-frame detection, or
`{"image":"frame.jpg","box":[x1,y1,x2,y2],"expected":"Red"}` for labelled
crop classification. Image paths are relative to the manifest. Expected states
are Red, Yellow, Green or Unknown. The report counts Unknown and false Green
separately; supplying labelled boxes does not evaluate the detector.

Initial public-sample check: 232 validation crops, 116 correct, 116 Unknown,
0 false Green. The official tutorial sample may overlap model training data;
this is an integration check, not independent road accuracy. Three old CARLA
crops all returned Unknown. Evidence: `logs/portable_signals/`.

For outdoor deployment, the remaining work is concrete: collect independent
full-frame sequences in the target region, label applicable lane signals and
pedestrians, assess misses/false permissions/latency including night and glare,
then fine-tune and calibrate thresholds on separate training/validation splits.
Replace simulator ego localization with measured localization and calibrated,
time-synchronized camera/LiDAR inputs; validate braking on a closed course.
Pedestrian YOLO/LiDAR fusion and ground-plane fallback remain prototypes,
not a validated outdoor crossing-occupancy system. No real vehicle should be
enabled by the current sample evaluation.

Sources: [Autoware model card](https://huggingface.co/AutowareFoundation/traffic_light_classifier),
[classifier preprocessing](https://github.com/autowarefoundation/autoware_universe/blob/main/perception/autoware_tensorrt_classifier/src/tensorrt_classifier.cpp).

## Trained RGB backend

`trained_rgb` is a separately trained 32x32 RGB → 96-unit ReLU → 4-class
MLP (Red, Yellow, Green, Unknown), with deterministic NumPy/Adam training and
NumPy inference. It does not require PyTorch or CUDA. Weights are stored in
`models/signal_rgb_mlp.npz`. It is not a fine-tuned copy of Autoware MobileNet;
the original backend remains available for comparison.

Static surveyed head projections provide image crops, avoiding reliance on COCO
YOLO for very small lights. Proposals are logged as `hd_map_projection`; their
proposal score of 1 is not a detector confidence. Colour and class confidence
come only from RGB pixels. Signal IDs/geometry are not classifier features.

Offline collection switches CARLA signals to label responsive head images.
Back-facing/nonresponsive heads and sampled backgrounds are Unknown. Intersection
949 is excluded from training and checkpoint selection; 961 is used for selection.
This is a grouped split, not a random split of adjacent frames. Autoware public
sample train images supplement CARLA crops; its val images are evaluated separately.
The public sample does not establish performance in the deployment region.

Close stopping positions (2.5–7 m), ±2-pixel shifts and exposure augmentation
addressed observed confidence drops as the car began moving. The 0.85 confidence
floor and 0.25 top-two margin were not lowered to pass the scenario.

For explicitly surveyed redundant heads, Unknown is abstention. Conflicting
visible colours still yield Unknown. Green requires two distinct agreeing head
ROIs and three fresh frames. This mode cannot release at a single-head junction.
Detector-box association continues to reject ambiguous signal identity.

Reproduce in `avlite-carla`, with one CARLA test client at a time:

```bash
python tools/collect_signal_training.py
python tools/collect_signal_training.py --near
OPENBLAS_NUM_THREADS=2 python tools/train_signal_classifier.py --real-archive /path/to/traffic_light_sample_dataset.tar.gz
OPENBLAS_NUM_THREADS=1 python tools/evaluate_signal_groups.py
python tools/check_sensor_perception.py --signal-backend trained_rgb --no-raw --colors Red Yellow Green Red --output logs/signal_training/final_lights
python tools/run_hdmap_scenarios.py --routes sensor_demo --repeats 2 --max-sim-time 180 --walking-pedestrian --pedestrians 3 --sensor-perception --signal-backend trained_rgb --output logs/signal_training/final_drives_complete
python tools/check_sensor_profile.py --profile sensor-road-trained --signal-backend trained_rgb --output logs/signal_training/executor_check.json
```

The real-image archive is the [official Autoware tutorial sample](https://autoware-files.s3.us-west-2.amazonaws.com/dataset/traffic_light_sample_dataset.tar.gz).
Collection, splits, confusion matrices and weights checksum are under
`logs/signal_training/`. Profile: `profiles/sensor-road-trained.yaml`. Use new
output directories for new runs; do not train/reload weights during a drive.

Limits: tested circular signals only. Arrow semantics, single-head junctions,
night/rain and calibration/localization errors remain unqualified. A real vehicle
still needs measured localization/extrinsics and independent full-frame road
validation. CARLA success does not establish deployment readiness.
