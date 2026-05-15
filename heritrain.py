from ultralytics import YOLO
import time
import torch

# ================= CONFIG =================
EPOCHS = 120
BATCH = 4
DEVICE = "cpu"

# ================= LOAD MODEL =================
print(f"Training on device: {DEVICE}")
model = YOLO("./yolov8n.pt")   # local weights (NO internet)

# ================= TIME TRACKING =================
start_time = time.time()
epoch_times = []

def format_time(seconds):
    h = int(seconds // 3600)
    m = int((seconds % 3600) // 60)
    s = int(seconds % 60)
    return f"{h:02d}:{m:02d}:{s:02d}"

# ================= TRAIN =================
results = model.train(
    data="data65.yaml",
    imgsz=640,
    epochs=EPOCHS,
    batch=BATCH,
    workers=2,
    device=DEVICE,
    optimizer="AdamW",
    lr0=0.001,
    cos_lr=True,

    mosaic=0,
    mixup=0,
    scale=0,
    translate=0,
    shear=0,
    perspective=0,
    hsv_h=0,
    hsv_s=0,
    hsv_v=0,

    project="runs_65m",
    name="yolov8n_cpu_eta",
    exist_ok=True
)

# ================= ETA REPORT =================
total_time = time.time() - start_time
avg_epoch_time = total_time / EPOCHS

print("\n================ TRAINING SUMMARY ================")
print(f"Total epochs       : {EPOCHS}")
print(f"Total training time: {format_time(total_time)}")
print(f"Avg time / epoch   : {format_time(avg_epoch_time)}")
print("==================================================")
print("✅ Training complete")
