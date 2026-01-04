import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# ノード名
NODE_NAME = 'color_filter_node'  # 必要に応じて変更すること

# 入力画像のエンコーディング形式
SUBSCRIBE_IMGMSG_ENCODING = 'bgr8' # 必要に応じて変更すること

# トピック名
SUBSCRIBE_TOPIC = '/image_src' # 変更しないこと
PUBLISH_BIN_TOPIC = '/image_bin' # 変更しないこと
PUBLISH_MASKED_TOPIC = '/image_masked' # 変更しないこと

cv_bridge = None
publisher_image_bin = None
publisher_image_masked = None
subscriber_image_src = None
node = None

# HSV の初期値（従来のスクリプトと同じ）
lower_hsv = np.array([100, 150, 0], dtype=np.uint8)
upper_hsv = np.array([140, 255, 255], dtype=np.uint8)

def _update_hsv_from_params():
    global lower_hsv, upper_hsv, node
    try:
        lh = int(node.get_parameter('lower_h').value)
        ls = int(node.get_parameter('lower_s').value)
        lv = int(node.get_parameter('lower_v').value)
        uh = int(node.get_parameter('upper_h').value)
        us = int(node.get_parameter('upper_s').value)
        uv = int(node.get_parameter('upper_v').value)
        lower_hsv = np.array([lh, ls, lv], dtype=np.uint8)
        upper_hsv = np.array([uh, us, uv], dtype=np.uint8)
        node.get_logger().info(f'HSV parameters updated: lower={lower_hsv.tolist()} upper={upper_hsv.tolist()}')
    except Exception as e:
        # node may be None or parameters missing
        if node is not None:
            node.get_logger().error(f'パラメータ読み取りエラー: {e}')

def _on_set_parameters(params):
    # パラメータ変更時に呼ばれるコールバック
    # コールバック内で params から直接値を読む（node.get_parameter() は遅延の原因）
    global lower_hsv, upper_hsv, node
    
    # 新しい値を取得
    new_values = {}
    changed = False
    for p in params:
        if p.name in ['lower_h', 'lower_s', 'lower_v', 'upper_h', 'upper_s', 'upper_v']:
            new_values[p.name] = int(p.value)
            changed = True
    
    if changed:
        try:
            # 変更されたパラメータと、変更されていないパラメータの現在値を組み合わせる
            lh = new_values.get('lower_h', lower_hsv[0])
            ls = new_values.get('lower_s', lower_hsv[1])
            lv = new_values.get('lower_v', lower_hsv[2])
            uh = new_values.get('upper_h', upper_hsv[0])
            us = new_values.get('upper_s', upper_hsv[1])
            uv = new_values.get('upper_v', upper_hsv[2])
            
            lower_hsv = np.array([lh, ls, lv], dtype=np.uint8)
            upper_hsv = np.array([uh, us, uv], dtype=np.uint8)
            if node is not None:
                node.get_logger().info(f'HSV parameters updated: lower={lower_hsv.tolist()} upper={upper_hsv.tolist()}')
            return SetParametersResult(successful=True)
        except Exception as e:
            if node is not None:
                node.get_logger().error(f'パラメータ更新処理でエラー: {e}')
            return SetParametersResult(successful=False)
    return SetParametersResult(successful=True)

def image_proc(src):
    # ここに画像処理コードを追加
    global lower_hsv, upper_hsv
    hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
    bin = cv2.inRange(hsv, lower_hsv, upper_hsv)
    masked = cv2.bitwise_and(src, src, mask=bin)
    return bin, masked

def image_proc_callback(msg):
  # 画像メッセージをOpenCVの画像形式（NumPy配列）に変換
  try:
      # カラー画像として変換 (bgr8形式を想定)
      cv_src = cv_bridge.imgmsg_to_cv2(msg, desired_encoding=SUBSCRIBE_IMGMSG_ENCODING)
  except Exception as e:
      if node is not None:
          node.get_logger().error(f'CvBridge変換エラー: {e}')
      else:
          rclpy.logging.get_logger(NODE_NAME).error(f'CvBridge変換エラー: {e}')
      return
  
  # --- 画像処理処理 ---
  cv_bin, cv_masked = image_proc(cv_src)
  
  # 処理後の画像をROS2のImageメッセージに変換してパブリッシュ
  try:
      bin_msg = cv_bridge.cv2_to_imgmsg(cv_bin, encoding='mono8')
      bin_msg.header = msg.header # タイムスタンプなどのヘッダ情報をコピー
      publisher_image_bin.publish(bin_msg)
      
      masked_msg = cv_bridge.cv2_to_imgmsg(cv_masked, encoding='bgr8')
      masked_msg.header = msg.header # タイムスタンプなどのヘッダ情報をコピー
      publisher_image_masked.publish(masked_msg)
  except Exception as e:
      if node is not None:
          node.get_logger().error(f'画像パブリッシュエラー: {e}')
      else:
          rclpy.logging.get_logger(NODE_NAME).error(f'画像パブリッシュエラー: {e}')

def main(args=None):
  
  global cv_bridge, publisher_image_bin, publisher_image_masked, subscriber_image_src, node

  # rclpyの初期化
  rclpy.init(args=args)

  # --- ノードの作成（クラス不使用のため、関数内で作成し、グローバル変数として保持） ---
  node = rclpy.create_node(NODE_NAME)

  node.get_logger().info('画像処理ノードを開始します...')

  # パラメータの宣言（デフォルトは従来の値）
  node.declare_parameter('lower_h', 100)
  node.declare_parameter('lower_s', 150)
  node.declare_parameter('lower_v', 0)
  node.declare_parameter('upper_h', 140)
  node.declare_parameter('upper_s', 255)
  node.declare_parameter('upper_v', 255)

  # パラメータの初期読み込み
  _update_hsv_from_params()

  # パラメータ変更時のコールバック登録
  node.add_on_set_parameters_callback(_on_set_parameters)

  # パブリッシャーの作成
  publisher_image_bin = node.create_publisher(Image, PUBLISH_BIN_TOPIC, 10)
  publisher_image_masked = node.create_publisher(Image, PUBLISH_MASKED_TOPIC, 10)

  # サブスクライバーの作成
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
      node.get_logger().info('Ctrl+Cが押されました。ノードをシャットダウンします...')
  finally:
      # --- ノード終了処理 ---
      node.destroy_node()
      rclpy.shutdown()

if __name__ == '__main__':
    main()