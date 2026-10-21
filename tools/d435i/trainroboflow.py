from pathlib import Path

from roboflow import Roboflow
from ultralytics import YOLO

import os


# Download the dataset (Roboflow skips the download if the folder already exists)
rf = Roboflow(api_key=os.environ["ROBOFLOW_API_KEY"])
project = rf.workspace("sdf-1byo9").project("excavator_bucket-twiii")
version = project.version(3)
dataset = version.download("yolov11")
data_yaml = Path(dataset.location) / "data.yaml"                

# Start from COCO-pretrained weights: n (fastest) / s / m (more accurate, slower)
# model = YOLO("yolo11s.pt")

model = YOLO("runs/detect/runs/excavator_bucket-4/weights/last.pt")

model.train(resume=True)


# model.train(
#     data=str(data_yaml),
#     epochs=300,
#     imgsz=640,
#     batch=8,        
#     device=0,        
#     patience=50,     
#     workers=4,
#     project="runs",
#     name="excavator_bucket",
# )

# Evaluate the best checkpoint on the validation set
best = YOLO(model.trainer.best)
metrics = best.val(data=str(data_yaml))
print(f"mAP50: {metrics.box.map50:.3f}  mAP50-95: {metrics.box.map:.3f}")



                