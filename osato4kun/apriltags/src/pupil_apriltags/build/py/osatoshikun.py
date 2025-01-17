import cv2
from pupil_apriltags import Detector
import numpy as np
import json
import socket
from scipy.spatial.transform import Rotation as Rot


#roboRIOからキャプチャした映像を取得
stream_url = "http://roborio-9494-frc.local:1181/?action=stream"
cap = cv2.VideoCapture(stream_url)
cap.set(cv2.CAP_PROP_FPS, 120)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

#cap = cv2.VideoCapture(0)


# サーバーソケットの設定
HOST = '127.0.0.1'  # サーバーのIPアドレス
PORT = 65432        # サーバーのポート番号
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen()

print(f"[*] Listening as {HOST}:{PORT}")

# クライアントの接続を待つ
client_socket, address = server_socket.accept()
print(f"[+] {address} is connected.")



# カメラの内部パラメータ(キャリブレーションで得たもの)
# camera_matrixはカメラの焦点距離と主点を含む
# dist_coeffsは歪み係数を含む。これらは画像から歪みを除去するのに使われる。
camera_matrix = np.array([[1.12886756e+03, 0.00000000e+00, 6.12666728e+02],
                          [0.00000000e+00, 1.13012861e+03, 3.75347099e+02],
                          [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
dist_coeffs = np.array([[1.52431294e-01, -1.20084759e+00, 2.91050653e-06, -3.46282507e-03, 2.20745254e+00]])

# AprilTag検出器の設定。使用するタグのファミリー、スレッド数、画像の処理パラメータ等を指定。
at_detector = Detector(families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

# AprilTagの実際の物理サイズ（メートル単位）この値はタグのポーズ推定に重要
tag_size = 0.151  

# 無限ループを使って、カメラからフレームを連続的に読み込み、処理する
while True:
    
    
    ret, frame = cap.read()  # カメラからフレームを読み込む
    if not ret:
        break  # フレームが正しく読み込めなかった場合はループを抜ける
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # フレームをグレースケールに変換

    # 歪み補正を適用。カメラの内部パラメータと歪み係数を使用。
    gray_undistorted = cv2.undistort(gray, camera_matrix, dist_coeffs)
    
    # 歪み補正された画像を使ってAprilTagを検出
    tags = at_detector.detect(gray_undistorted, estimate_tag_pose=True, 
                              camera_params=(camera_matrix[0,0], camera_matrix[1,1], camera_matrix[0,2], camera_matrix[1,2]), 
                              tag_size=tag_size)
    # AprilTagの割り当て
    AprilTags = {1:"BlueSorce(R)", 2:"BlueSorce(L)", 3:"RedSpeaker(R)", 4:"RedSpeaker(Center)", 5:"RedAmp", 6:"BlueAmp", 7:"BlueSpeaker(R)", 8:"BlueSpeaker(Center)", 9:"RedSource(R)", 10:"RedSource(L)", 11:"RedStage(2o'clock)", 12:"RedStage(10o'clock)", 13:"RedStage(6o'clock)", 14:"BlueStage(6o'clock)", 15:"BlueStage(2o'clock)", 16:"BlueSorce(10o'clock)"}
    # 検出された各タグに対して処理を実施
    for tag in tags:
        # タグのIDによって色を設定
        if tag.tag_id in [1, 2, 6, 7, 8, 14, 15, 16]:  # 青色のタグ
            color = (255, 0, 0)  # BGR形式で青色
        else:  # 赤色のタグ
            color = (0, 0, 255)  # BGR形式で赤色

        # タグの角度（オイラー角）を計算
        R = tag.pose_R  # 回転行列
        sy = np.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
        singular = sy < 1e-6
        if not singular:
            x = np.arctan2(R[2,1] , R[2,2])
            y = np.arctan2(-R[2,0], sy)
            z = np.arctan2(R[1,0], R[0,0])
        else:
            x = np.arctan2(-R[1,2], R[1,1])
            y = np.arctan2(-R[2,0], sy)
            z = 0
            
        rotation_matrix = tag.pose_R
        rotation = Rot.from_matrix(rotation_matrix)
        quaternion = rotation.as_quat()  # [x, y, z, w] の形式
        
        # 四元数からオイラー角への変換を行う
        rotation_matrix = tag.pose_R
        rotation = Rot.from_matrix(rotation_matrix)  # 回転行列からRotationオブジェクトを作成
        euler_angles = rotation.as_euler('xyz', degrees=True)  # オイラー角へ変換

        
        # タグの四隅の座標を整数型で取得
        corners = np.array(tag.corners).astype(int)
        
        # タグの四隅に指定した色の線を描画してタグを視覚的に表示
        cv2.polylines(frame, [corners], isClosed=True, color=color, thickness=4)
        
        # タグのIDとカメラからの距離を計算し、フレーム上にテキストとして表示
        distance = np.linalg.norm(tag.pose_t / 2.5)  # 並進ベクトルを使用して距離を計算
        cv2.putText(frame, f"Object: {AprilTags[tag.tag_id]}", (corners[0][0] + 10, corners[0][1] - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        cv2.putText(frame, f"Distance: {distance:.2f} m", (corners[0][0] + 10, corners[0][1] - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
    # オイラー角のデータをフレーム上に表示
        cv2.putText(frame, f"Roll: {euler_angles[0]:.2f}, Pitch: {euler_angles[1]:.2f}", (corners[0][0] + 10, corners[0][1] - 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

    # 処理が完了したフレームをウィンドウで表示
    frame = cv2.resize(frame,dsize=(640,480))
    cv2.imshow('AprilTag Detection', frame)
    
        # 検出されたタグの情報をJSON形式でクライアントに送信
    try:
        # JSON形式でクライアントに送信するためのデータ準備
        tags_data = [{
            "id": tag.tag_id,
            "distance": np.linalg.norm(tag.pose_t / 2.5),
            "euler_angles": {"roll": euler_angles[0], "pitch": euler_angles[1], "yaw": euler_angles[2]}} for tag in tags]
        
        message = json.dumps(tags_data) + "\n"
        client_socket.sendall(message.encode('utf-8'))
        
    except BrokenPipeError:
        # クライアントが接続を閉じた場合
        print("[-] Client disconnected. Waiting for the next connection...")
        client_socket, address = server_socket.accept()
        print(f"[+] {address} is connected.")
        continue

    # 'q'キーが押されたらループ（とプログラム）を終了
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# カメラデバイスを解放し、オープンしたウィンドウをすべて閉じる
cap.release()
cv2.destroyAllWindows()