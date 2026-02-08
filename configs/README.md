# Configuration Files

This directory contains configuration files for various components of the MariSense project.

## Available Configurations

### Dataset Configuration

- [`dataset_config.yaml`](dataset_config.yaml) - General dataset configuration
- [`splits_config.yaml`](splits_config.yaml) - Train/val/test split configuration

### Task-Specific Configurations

#### Object Detection

- [`detection/yolov8_config.yaml`](detection/yolov8_config.yaml) - YOLOv8 training configuration
- [`detection/faster_rcnn_config.yaml`](detection/faster_rcnn_config.yaml) - Faster R-CNN configuration
- [`detection/retinanet_config.yaml`](detection/retinanet_config.yaml) - RetinaNet configuration

#### Object Tracking

- [`tracking/sort_config.yaml`](tracking/sort_config.yaml) - SORT tracker configuration
- [`tracking/deepsort_config.yaml`](tracking/deepsort_config.yaml) - DeepSORT configuration
- [`tracking/bytetrack_config.yaml`](tracking/bytetrack_config.yaml) - ByteTrack configuration

#### Odometry

- [`odometry/vo_config.yaml`](odometry/vo_config.yaml) - Visual odometry configuration
- [`odometry/vio_config.yaml`](odometry/vio_config.yaml) - Visual-inertial odometry configuration
- [`odometry/lidar_config.yaml`](odometry/lidar_config.yaml) - LiDAR odometry configuration

### Evaluation Configuration

- [`evaluation_config.yaml`](evaluation_config.yaml) - Evaluation metrics and settings

### Visualization Configuration

- [`visualization_config.yaml`](visualization_config.yaml) - Visualization settings

## Usage

### Loading Configuration in Python

```python
import yaml

# Load configuration
with open('configs/dataset_config.yaml', 'r') as f:
    config = yaml.safe_load(f)

# Access configuration values
data_path = config['data']['path']
batch_size = config['training']['batch_size']
```

### Using Configuration with CLI

```bash
# Train with specific configuration
python tools/train_detector.py --config configs/detection/yolov8_config.yaml

# Evaluate with configuration
python tools/evaluate.py --config configs/evaluation_config.yaml
```

### Configuration Override

You can override configuration values via command line:

```bash
python tools/train_detector.py \
    --config configs/detection/yolov8_config.yaml \
    --batch-size 16 \
    --learning-rate 0.001
```

## Configuration Format

All configuration files use YAML format for readability and ease of use.

### Example Configuration Structure

```yaml
# General settings
project_name: "MariSense"
version: "1.0.0"

# Data settings
data:
  path: "data/"
  split: "train"
  batch_size: 32
  num_workers: 4

# Model settings
model:
  name: "yolov8"
  backbone: "resnet50"
  pretrained: true

# Training settings
training:
  epochs: 100
  learning_rate: 0.001
  optimizer: "adam"
  scheduler: "cosine"

# Evaluation settings
evaluation:
  metrics: ["mAP", "precision", "recall"]
  iou_threshold: 0.5
```

## Creating Custom Configurations

To create a custom configuration:

1. Copy an existing configuration file
2. Modify the parameters as needed
3. Save with a descriptive name
4. Use with your training/evaluation scripts

## Configuration Validation

Validate your configuration files:

```bash
python tools/validate_config.py --config configs/your_config.yaml
```

## Best Practices

- Keep configurations version controlled
- Use descriptive names for custom configs
- Document any non-standard parameters
- Validate configs before running experiments
- Store experiment-specific configs separately
