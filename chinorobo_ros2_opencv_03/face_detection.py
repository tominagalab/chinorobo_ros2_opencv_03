import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# ノード名
NODE_NAME = 'face_detection_node'  # 必要に応じて変更すること

# 入力画像のエンコーディング形式
SUBSCRIBE_IMGMSG_ENCODING = 'bgr8' # 必要に応じて変更すること
# 出力画像のエンコーディング形式
PUBLISH_IMGMSG_ENCODING = 'bgr8' # 必要に応じて変更すること

# トピック名
SUBSCRIBE_TOPIC = '/image_src' # 変更しないこと
PUBLISH_TOPIC = '/image_dst' # 変更しないこと

cv_bridge = None
publisher_image_dst = None
subscriber_image_src = None

def image_proc(src):
    # ここに画像処理コードを追加
    
    # グレースケール変換
    grayscale = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
    
    # カスケード分類器の読み込み
    cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    
    # 顔検出の実行
    faces = cascade.detectMultiScale(grayscale, scaleFactor=1.1, minNeighbors=3, minSize=(30, 30))
    
    # 検出された顔に矩形を描画
    dst = src.copy()
    for (x, y, w, h) in faces:
        cv2.rectangle(dst, (x, y), (x + w, y + h), (0, 255, 0), 2)
    
    return dst

def image_proc_callback(msg):
  # 画像メッセージをOpenCVの画像形式（NumPy配列）に変換
  try:
      # カラー画像として変換 (bgr8形式を想定)
      cv_src = cv_bridge.imgmsg_to_cv2(msg, desired_encoding=SUBSCRIBE_IMGMSG_ENCODING)
  except Exception as e:
      rclpy.logging.get_logger(NODE_NAME).error(f'CvBridge変換エラー: {e}')
      return
  
  # --- 画像処理処理 ---
  cv_dst = image_proc(cv_src)
  
  # 処理後の画像をROS2のImageメッセージに変換してパブリッシュ
  try:
      dst_msg = cv_bridge.cv2_to_imgmsg(cv_dst, encoding=PUBLISH_IMGMSG_ENCODING)
      dst_msg.header = msg.header # タイムスタンプなどのヘッダ情報をコピー
      publisher_image_dst.publish(dst_msg)
  except Exception as e:
      rclpy.logging.get_logger(NODE_NAME).error(f'画像パブリッシュエラー: {e}')  

def main(args=None):
  
  global cv_bridge, publisher_image_dst, subscriber_image_src
  
  # rclpyの初期化
  rclpy.init(args=args)

  # --- ノードの作成（クラス不使用のため、関数内で作成し、グローバル変数として保持） ---
  # rclpy.create_nodeを使ってノードを作成
  node = rclpy.create_node(NODE_NAME)
  
  rclpy.logging.get_logger(NODE_NAME).info('画像処理ノードを開始します...')
  
  #パブリッシャーの作成
  publisher_image_dst = node.create_publisher(Image, PUBLISH_TOPIC, 10)

  #サブスクライバーの作成
  #トピックをサブスクライブし、image_callback関数を呼び出す
  subscriber_image_src = node.create_subscription(
      Image,
      SUBSCRIBE_TOPIC,
      image_proc_callback,
      10
  )
  
  # CvBridgeの初期化
  cv_bridge = CvBridge()

  # Ctrl+Cによる安全なシャットダウンのためのtry-exceptブロック
  try:
      rclpy.spin(node)
  except KeyboardInterrupt:
      rclpy.logging.get_logger(NODE_NAME).info('Ctrl+Cが押されました。ノードをシャットダウンします...')
  finally:
      # --- ノード終了処理 ---
      node.destroy_node()
      rclpy.shutdown()

if __name__ == '__main__':
    main()