from ultralytics import YOLO
import os

def run_inference(
    model_path: str,
    source: str,
    output_dir: str = "inference_results",
    imgsz: int = 640,
    conf: float = 0.25,
    iou: float = 0.45,
    device: str = "cpu"
):
    """
    YOLOv8 推論を実行し、結果を保存する関数。

    Args:
        model_path: 学習済みモデルファイルへのパス (例: 'results/test1_run/weights/best.pt')
        source: 推論対象 (画像ファイル、フォルダ、あるいはビデオファイル)
        output_dir: 推論結果の出力先フォルダ (自動で作成されます)
        imgsz: 推論時の入力リサイズサイズ
        conf: 最低信頼度しきい値 (0.0–1.0)
        iou: NMS の IoU 閾値 (0.0–1.0)
        device: 利用デバイス ('cpu' or 'cuda')
    """
    # 出力ディレクトリ作成
    os.makedirs(output_dir, exist_ok=True)

    # モデル読み込み
    model = YOLO(model_path)

    # 推論実行
    results = model.predict(
        source=source,
        imgsz=imgsz,
        conf=conf,
        iou=iou,
        device=device,
        save=True,             # 結果を画像として保存
        save_txt=False,        # ラベルテキストを保存するなら True
        project=output_dir,    # 保存先ディレクトリ
        name="run",            # サブフォルダ名
        exist_ok=True          # 同名フォルダがあっても上書き
    )

    print(f"Inference done. Results saved to: {os.path.join(output_dir, 'run')}")

if __name__ == "__main__":
    # 学習済みモデルファイル
    model_file = "results/test1_run/weights/best.pt"
    # 推論したい画像またはディレクトリ
    # 例: 単一画像 -> "input.jpg"
    #     画像フォルダ -> "data/images/"
    #     ビデオファイル -> "input.mp4"
    source_path = "some_image.jpg"

    run_inference(
        model_path=model_file,
        source=source_path,
        output_dir="inference_results",
        imgsz=640,
        conf=0.25,
        iou=0.45,
        device="cpu"  # GPU を使う場合は "cuda:0" などに
    )
