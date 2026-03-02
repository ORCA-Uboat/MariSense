# Object Tracking

## Overview

The object tracking component of MariSense provides image sequences with frame-by-frame annotations for tracking maritime objects. The dataset includes synchronized thermal and low-light camera images with bounding box annotations that maintain object identities across frames.

### Dataset download

The dataset can be downloaded from the [official website](https://orca-tech.cn/datasets/MariSense/Introduction). We also provide [some sample data](https://nas.orca-tech.cn:5000/sharing/lWI1Hq2np)

### Dataset Composition

The tracking dataset consists of 300 sequences captured under various maritime conditions.

| Sequence           | Scene   | Lighting | Weather | Sea State |
| ------------------ | ------- | -------- | ------- | --------- |
| pic_10_31_14-15-22 | Fairway | Daytime  | Cloudy  | 2         |
| ...                | ...     | ...      | ...     | ...       |

*Note: A metadata file is provided listing all sequences with their corresponding scene, lighting, weather, and sea state conditions.*

## Data Storage Format

### Directory Structure

The tracking data is organized with images and annotations stored separately:

```
tracking_data/
├── images/                          # Image data
│   ├── pic_10_31_14-15-22/         # Sequence folder (named by timestamp)
│   │   ├── thermal/                 # Thermal camera images
│   │   │   ├── 2025_10_31_14-15-34-197.png
│   │   │   ├── 2025_10_31_14-15-34-297.png
│   │   │   └── ...
│   │   └── lowlight/                # Low-light camera images
│   │       ├── 2025_10_31_14-15-34-197.png
│   │       ├── 2025_10_31_14-15-34-297.png
│   │       └── ...
│   ├── pic_10_31_15-20-30/         # Another sequence
│   │   ├── thermal/
│   │   └── lowlight/
│   └── ...
│
├── annotations/                     # Annotation data
│   ├── pic_10_31_14-15-22/         # Sequence folder (matches image folder)
│   │   ├── thermal/                 # Thermal image annotations
│   │   │   ├── 2025_10_31_14-15-34-197.xml
│   │   │   ├── 2025_10_31_14-15-34-297.xml
│   │   │   └── ...
│   │   └── lowlight/                # Low-light image annotations
│   │       ├── 2025_10_31_14-15-34-197.xml
│   │       ├── 2025_10_31_14-15-34-297.xml
│   │       └── ...
│   ├── pic_10_31_15-20-30/
│   │   ├── thermal/
│   │   └── lowlight/
│   └── ...
│
└── metadata.csv                     # Sequence metadata (scene, lighting, weather, sea state)
```

### Naming Conventions

- **Sequence Folders**: Named by timestamp format `pic_MM_DD_HH-MM-SS`
- **Image Files**: Named by timestamp `YYYY_MM_DD_HH-MM-SS-mmm.png` (mmm = milliseconds)
- **Annotation Files**: Same name as corresponding image but with `.xml` extension
- **Camera Types**: `thermal/` for thermal camera, `lowlight/` for low-light camera

### Image Specifications

- **Lowlight Image Resolution**: 1920 × 1080 pixels
- **Infrared Image Resolution**: 640 × 512 pixels
- **Format**: PNG
- **Channels**: 3 (RGB representation)
- **Synchronization**: Thermal and low-light images are time-synchronized

## Tracking Annotations

### Annotation Format

Annotations are stored in **XML format**, with one XML file per image frame. Each annotation file contains bounding boxes for all objects visible in that frame.

#### XML Structure

```xml
<?xml version='1.0' encoding='utf-8'?>
<annotation>
    <folder>pic_10_31_14-15-22</folder>
    <filename>2025_10_31_14-15-34-197.png</filename>
    <path>pic_10_31_14-15-22/2025_10_31_14-15-34-197.png</path>
    <source>
        <database>Unknown</database>
    </source>
    <size>
        <width>1920</width>
        <height>1080</height>
        <depth>3</depth>
    </size>
    <segmented>0</segmented>
    <object>
        <name>ship24</name>
        <pose>Unspecified</pose>
        <truncated>0</truncated>
        <difficult>0</difficult>
        <bndbox>
            <xmin>441</xmin>
            <ymin>607</ymin>
            <xmax>455</xmax>
            <ymax>611</ymax>
        </bndbox>
    </object>
    <object>
        <name>other1</name>
        <pose>Unspecified</pose>
        <truncated>0</truncated>
        <difficult>0</difficult>
        <bndbox>
            <xmin>413</xmin>
            <ymin>607</ymin>
            <xmax>419</xmax>
            <ymax>615</ymax>
        </bndbox>
    </object>
    <object>
        <name>ship21</name>
        <pose>Unspecified</pose>
        <truncated>0</truncated>
        <difficult>0</difficult>
        <bndbox>
            <xmin>176</xmin>
            <ymin>631</ymin>
            <xmax>188</xmax>
            <ymax>636</ymax>
        </bndbox>
    </object>
    <object>
        <name>ship23</name>
        <pose>Unspecified</pose>
        <truncated>0</truncated>
        <difficult>0</difficult>
        <bndbox>
            <xmin>127</xmin>
            <ymin>637</ymin>
            <xmax>139</xmax>
            <ymax>640</ymax>
        </bndbox>
    </object>
</annotation>
```

#### Field Descriptions

**File Information**:

- `<folder>`: Sequence folder name
- `<filename>`: Image filename
- `<path>`: Relative path to image
- `<size>`: Image dimensions (width, height, depth)

**Object Information**:

- `<name>`: Object class and track ID (format: `{class}{id}`)
  - Example: `ship24` means object class is "ship" with track ID 24
  - Example: `other1` means object class is "other" with track ID 1
- `<pose>`: Object pose (typically "Unspecified")
- `<truncated>`: Whether object is truncated at image boundary (0 or 1)
- `<difficult>`: Whether object is difficult to detect (0 or 1)
- `<bndbox>`: Bounding box coordinates
  - `<xmin>`, `<ymin>`: Top-left corner coordinates
  - `<xmax>`, `<ymax>`: Bottom-right corner coordinates

### Object Naming Convention

The `<name>` field encodes both the object class and its unique track ID:

**Format**: `{class_name}{track_id}`

**Examples**:

- `ship24` → Class: "ship", Track ID: 24
- `buoy12` → Class: "buoy", Track ID: 12
- `other1` → Class: "other", Track ID: 1

**Track ID Consistency**: The same track ID is maintained across all frames where the object appears, enabling multi-object tracking.

### Object Categories

- **ship**: Maritime vessels
- **buoy**: Navigation buoys
- **other**: Other maritime objects

