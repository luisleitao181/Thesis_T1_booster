# YOLO Training – Environment Feature Detection for Booster T1

This module contains the full pipeline used to train a YOLO-based detector for environmental feature recognition using the Booster T1 RGB camera.

The repository contains only source code and experiment documentation.  
Large datasets, recordings, and trained weights are intentionally excluded.

---

# 1. Pipeline Overview

The complete perception pipeline followed these stages:

1. Data recording (ROS2 camera stream from Booster T1)
2. Frame extraction from recordings
3. Duplicate and near-duplicate removal
4. Annotation (Roboflow)
5. Dataset export (YOLOv8 format)
6. Training (Google Colab – GPU)
7. Model evaluation and inference testing
8. Deployment on robot

---

# 2. Folder Structure

## /recordings
Contains raw data collected from the robot:

- ROS2 bags
- Extracted videos
- Raw image sequences

⚠️ NOT versioned in Git (ignored via .gitignore)  
Reason: Large storage size.

---

## /dataset_tools

Contains dataset preprocessing scripts:

- remove_duplicates.py
- remove_similar.py

These scripts perform perceptual hash (pHash) filtering to remove redundant frames.

Similarity threshold used:
18

Purpose:
Reduce dataset bias and avoid training on highly similar frames.

✔ Versioned in Git  
❌ Processed image datasets are NOT versioned

---

## /model_1

Documents the first trained YOLO model used in this thesis.

This folder does NOT store trained weights or heavy outputs.
It only documents the experiment configuration.

---

# 3. Dataset Information

Dataset name:
T1_environment_features.v2i.yolov8

Dataset split:
- 96 training images
- 27 validation images
- 14 test images

Augmentations applied:
- ±7° rotation
- ±25% brightness
- ±10% exposure
- 2x augmentation

Total images after augmentation:
233

Annotation platform:
Roboflow

The dataset archive (.zip) and raw images are NOT stored in this repository.

---

# 4. Training Configuration

Architecture:
YOLOv8 (Ultralytics)

Training environment:
Google Colab (GPU runtime)

Colab notebook link:
https://colab.research.google.com/drive/1fIqpzECHnBSKzCEH5csc5kVlaU3_FpHa?usp=sharing

Trained weights:
best.pt (stored externally)

---

# 5. Outputs (Not Versioned)

The following files are intentionally excluded from Git:

- best.pt
- training runs/
- inference result videos (.avi, .mp4)
- exported TensorRT .engine files

Reason:
These files are large and can be regenerated from the dataset and training configuration.

---

# 6. Reproducibility Notes

To reproduce Model 1:

1. Record data from Booster T1 camera
2. Run remove_duplicates.py and remove_similar.py
3. Upload filtered dataset to Roboflow
4. Export dataset in YOLOv8 format
5. Train using the provided Colab notebook
6. Export best.pt

This repository contains all scripts necessary for dataset preprocessing and documentation of training parameters.

---

# 7. Status

Model 1 represents the baseline perception model for the thesis.

Future improvements may include:
- Additional dataset collection
- Multi-class expansion
- Improved augmentation strategy
- Quantitative performance evaluation
- Deployment optimization for real-time inference