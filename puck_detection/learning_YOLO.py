import os
from ultralytics import YOLO

# スクリプトが置いてあるディレクトリ
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# データセットディレクトリと data.yaml のパス
DATASET_DIR = os.path.join(BASE_DIR, "test1-2")
DATA_YAML   = os.path.join(DATASET_DIR, "data.yaml")

# 存在チェック
if not os.path.isfile(DATA_YAML):
    raise FileNotFoundError(f"{DATA_YAML} が見つかりません。")

# モデルロード＆学習
model = YOLO("yolov8n.pt")
model.train(
    data=DATA_YAML,
    epochs=150, # 50
    imgsz=640,
    batch=32, # 16
    workers=0,
    project="results",
    name="test1_run"
)
