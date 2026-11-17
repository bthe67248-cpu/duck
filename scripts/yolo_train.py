r"""yolo_train.py -- fine-tune YOLOv8-nano on the rendered images and export it to ONNX.

Usage:  .venv\Scripts\python.exe scripts\yolo_train.py [--epochs 30]
Starts from the COCO-pretrained yolov8n.pt (downloaded by Ultralytics on first use), trains on the
CPU at 320 px, then writes yolo\duck_yolo.onnx, which the ROS 2 node runs with onnxruntime.
"""
import argparse, shutil
from pathlib import Path
from ultralytics import YOLO

ROOT = Path(__file__).resolve().parents[1]
if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--epochs", type=int, default=30); a = ap.parse_args()
    model = YOLO(str(ROOT / "yolo" / "yolov8n.pt"))
    model.train(data=str(ROOT / "yolo" / "dataset" / "data.yaml"), epochs=a.epochs, imgsz=320, batch=32, device="cpu",
                workers=0, project=str(ROOT / "yolo" / "runs"), name="duck", exist_ok=True, plots=False, verbose=False,
                fliplr=0.5, mosaic=0.5)
    best = YOLO(str(ROOT / "yolo" / "runs" / "duck" / "weights" / "best.pt"))
    met = best.val(data=str(ROOT / "yolo" / "dataset" / "data.yaml"), imgsz=320, device="cpu", workers=0, plots=False, verbose=False)
    onnx = best.export(format="onnx", imgsz=(256, 320), opset=17, simplify=True)     # 240 rows padded to 256
    shutil.copy2(onnx, ROOT / "yolo" / "duck_yolo.onnx")
    lines = [f"val images: {len(list((ROOT / 'yolo' / 'dataset' / 'images' / 'val').glob('*.jpg')))}",
             f"mAP50 all {met.box.map50:.3f}   mAP50-95 all {met.box.map:.3f}"]
    for i, n in best.names.items():
        try:
            p, r, m50, m = met.box.class_result(list(met.box.ap_class_index).index(i))
            lines.append(f"{n:7s} precision {p:.3f} recall {r:.3f} mAP50 {m50:.3f}")
        except ValueError:
            lines.append(f"{n:7s} not present in val")
    (ROOT / "yolo" / "val_metrics.txt").write_text("\n".join(lines) + "\n")
    print("\n".join(lines)); print("exported", ROOT / "yolo" / "duck_yolo.onnx")
