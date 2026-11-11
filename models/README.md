# Detector weights

Run `python install.py` from the plugin root. External ONNX weights are not
bundled in this repository.

The installed file is YOLOv8n COCO, ONNX export distributed by
https://huggingface.co/webml/yolov8n/blob/main/onnx/yolov8n.onnx .
SHA256: `190ba5f1e61411a001683e349d6b2cdb0804c0dc67a5e34cd8ff6fd00ee54b4d`.

Upstream architecture/model documentation: https://docs.ultralytics.com/models/yolov8/ .
Weights are an external artifact, not original AVLite code; consult the upstream
Ultralytics AGPL-3.0/enterprise licensing terms before redistributing them.
# Autoware traffic-light classifier

`traffic_light_mobilenetv2.onnx`: official AutowareFoundation
`traffic_light_classifier_mobilenetv2_batch_1.onnx`, pinned tag `v4.0`.
Source: https://huggingface.co/AutowareFoundation/traffic_light_classifier
SHA256: `455b71b3b20d3a96aa0e49f32714ba50421f668a2f9b9907c30b1346ac8a3703`.
Model card declares Apache-2.0. Install with `python install.py --autoware`.
Upstream training uses Japanese traffic signals; geographic generalization is
not established. See SENSOR_PERCEPTION.md for adapter limitations and evidence.


# Trained RGB model

`signal_rgb_mlp.npz` contains locally trained NumPy MLP weights (32x32 RGB,
96 hidden units, four classes). SHA256:
`b46046d3b11e74d3390fa34de04d5dbd3377714356952b1c9e40f062cd44b712`.
Reproduce using `tools/collect_signal_training.py` and
`tools/train_signal_classifier.py`; see [SENSOR_PERCEPTION.md](../SENSOR_PERCEPTION.md) for grouped splits,
augmentation, source data and limits. This is a new model, not an Autoware
MobileNet checkpoint. Full training metrics: `logs/signal_training/training_report.json`.
