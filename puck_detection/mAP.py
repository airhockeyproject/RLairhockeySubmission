from ultralytics import YOLO

# モデルの読み込み
model_path = "results/test1_run5/weights/best.pt"
model = YOLO(model_path)

# 評価対象のデータセット（test: test/images が含まれる必要あり）
data_yaml_path = "test1-2/data.yaml"

# testセットで評価（split='test'）
results = model.val(data=data_yaml_path, split="test",workers=0)

# 指標を取得（1クラス前提で .p[0], .r[0] を取得）
box_precision = results.box.p[0]     # Precision
box_recall = results.box.r[0]        # Recall
map_50 = results.box.map50           # mAP@0.5

# 出力
print("===== Evaluation on test set =====")
print(f"Box Precision (P)   : {box_precision:.4f}")
print(f"Box Recall    (R)   : {box_recall:.4f}")
print(f"mAP@0.5              : {map_50:.4f}")
