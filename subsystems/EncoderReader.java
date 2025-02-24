package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EncoderReader {

    public static void main(String[] args) {
        // エンコーダーのCAN IDを指定します (例: 3)。
        int canID = 3;

        try (CANcoder encoder = new CANcoder(canID)) {
            // エンコーダーの初期化
            System.out.println("CANcoder 初期化完了: CAN ID " + canID);

            // 無限ループでエンコーダーの値を更新
            while (true) {
                // 最新の位置情報を取得
                // StatusSignal<Double> positionSignal = encoder.getPosition();
                // positionSignal.refresh(); // 値を更新
                // double position = positionSignal.getValue(); // 値を取得

                StatusSignal<Double> positionSignal = encoder.getPosition();
                positionSignal.refresh(); // 値を更新
                double position = positionSignal.getValue(); // 値を取得

                position = position % 10;
                // SmartDashboard に出力
                SmartDashboard.putNumber("エンコーダー位置", position);

                // デバッグ用にコンソールにも出力
                System.out.println("エンコーダー位置: " + position);

                // 更新間隔を短くしすぎないためにスリープを入れる
                Thread.sleep(100); // 100ミリ秒
            }
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("エンコーダーの読み取り中にエラーが発生しました。");
        }
    }
}

