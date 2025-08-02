import os
from dynamixel_sdk import * # Dynamixel SDKのすべてをインポート

# ----- 設定項目 -----
# 通信ポートとボーレートはご自身の環境に合わせてください
DEVICE_NAME = '/dev/ttyUSB0'
BAUDRATE = 1000000

# Dynamixelのプロトコルバージョン
PROTOCOL_VERSION = 2.0

# 制御するDynamixelのIDと、目標角度
DXL_ID = 1  # 動かしたいサーボのID
GOAL_POSITION = 2048 # 0から4095の間の値。2048はだいたい中央の位置

# --------------------

# ポートとパケットハンドラの初期化
portHandler = PortHandler(DEVICE_NAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# ポートを開く
if not portHandler.openPort():
    print("ポートを開けませんでした。")
    quit()
print(f"{DEVICE_NAME} を開きました。")

# ボーレートを設定
if not portHandler.setBaudRate(BAUDRATE):
    print("ボーレートの変更に失敗しました。")
    quit()
print(f"ボーレートを {BAUDRATE} bpsに設定しました。")

# --- ここからが制御の本番 ---

# トルクをONにする (アドレス11, 1バイト, 値1)
# これをしないとサーボは動きません
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, 64, 1)
if dxl_comm_result != COMM_SUCCESS:
    print(f"トルクON失敗: {packetHandler.getTxRxResult(dxl_comm_result)}")
elif dxl_error != 0:
    print(f"トルクONエラー: {packetHandler.getRxPacketError(dxl_error)}")
else:
    print(f"ID {DXL_ID} のトルクをONにしました。")

# 目標位置へ指令を送る (アドレス116, 4バイト, 値GOAL_POSITION)
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, 116, GOAL_POSITION)
if dxl_comm_result != COMM_SUCCESS:
    print(f"位置指令失敗: {packetHandler.getTxRxResult(dxl_comm_result)}")
elif dxl_error != 0:
    print(f"位置指令エラー: {packetHandler.getRxPacketError(dxl_error)}")
else:
    print(f"ID {DXL_ID} に目標位置 {GOAL_POSITION} を指令しました。")

# 少し待つ (サーボが動く時間)
import time
time.sleep(2)

# トルクをOFFにする (安全のため)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, 64, 0)
print(f"ID {DXL_ID} のトルクをOFFにしました。")

# ポートを閉じる
portHandler.closePort()
print("ポートを閉じました。")
