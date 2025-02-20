
// package frc.robot;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.Joystick;

// public class Robot extends TimedRobot {
//     // 左右のモーターを制御するTalonSRXの定義
//     private final TalonSRX leftMotor1 = new TalonSRX(5); // 左モーター1
//     // private final TalonSRX leftMotor2 = new TalonSRX(4); // 左モーター2
//     private final TalonSRX rightMotor1 = new TalonSRX(3); // 右モーター1
//     // private final TalonSRX rightMotor2 = new TalonSRX(2); // 右モーター2

//     // ロジテックコントローラーの定義
//     private final Joystick controller = new Joystick(0);

//     @Override
//     public void teleopPeriodic() {
//         // コントローラーのスティック入力を取得
//         double leftSpeed = -controller.getRawAxis(1);  // 左スティックのY軸
//         double rightSpeed = controller.getRawAxis(5); // 右スティックのY軸
//         // 左右のモーターに速度を設定（右側を反転）
//         leftMotor1.set(ControlMode.PercentOutput, leftSpeed);
//         // leftMotor2.set(ControlMode.PercentOutput, -leftSpeed);
//         rightMotor1.set(ControlMode.PercentOutput, rightSpeed);
//         // rightMotor2.set(ControlMode.PercentOutput, -rightSpeed);

//         // 速度に応じた条件付き処理（例: 最小値チェック）
//         if (Math.abs(leftSpeed) > 0.3 || Math.abs(rightSpeed) > 0.3) {
//             leftMotor1.set(ControlMode.PercentOutput, leftSpeed);
//             // leftMotor2.set(ControlMode.PercentOutput, -leftSpeed);
//             rightMotor1.set(ControlMode.PercentOutput, rightSpeed);
//             // rightMotor2.set(ControlMode.PercentOutput, -rightSpeed);
//         }
//     }
// }


// This is the latest version 2/10/2025

package frc.robot;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.fasterxml.jackson.databind.annotation.EnumNaming;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.TmedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.cameraserver.CameraServer;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;


import com.ctre.phoenix.motorcontrol.ControlMode;
// import april.AprilTagClientGUI;
public class Robot extends TimedRobot {
    private PWMSparkMax leftMotor1, leftMotor2, rightMotor1, rightMotor2;
    private PWMSparkMax rightNeoMotor, leftNeoMotor;
    private Joystick joystick;
    private Servo myServo;
    private TalonFX krakenMotor;
    private PhotonCamera camera;
    private DifferentialDrive drive;

    
    TalonSRX talonSRX = new TalonSRX(1);
    TalonSRX rightArmMotor = new TalonSRX(2);
    TalonSRX leftArmMotor = new TalonSRX(3);
    // ポート番号
    private final int rightNeoMotorPort = 0;
    private final int leftNeoMotorPort = 1;
    private final int leftMotor1Port = 2;
    private final int leftMotor2Port = 3;
    private final int rightMotor1Port = 4;
    private final int rightMotor2Port = 5;
    private final int servoPort = 6;
    private final int joystickPort = 0;
    private final Timer m_timer = new Timer();
    private double shootSpeed = 0.4;
    // AprilTagClientGUI guiInstance;
    private final double kDriveTick2Feet = 1.0 / 4096 * 6 * Math.PI / 12;
    // kDriveTick2Feet が、距離を測るための変数で、エンコーダーのギア数とか色々計算してくれてる
    // 6という数字について: ホイールの直径（この例では6インチ）とπを掛け合わせて、ホイールの円周をインチで計算します。ホイールが1回転すると、ロボットはこの距離だけ進むことになります。
    private static final String kLeftAuto = "leftSide";
    private static final String kRightAuto = "rightSide";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();
    @Override
    public void robotInit() {
        rightNeoMotor = new PWMSparkMax(rightNeoMotorPort);
        leftNeoMotor = new PWMSparkMax(leftNeoMotorPort);
        leftMotor1 = new PWMSparkMax(leftMotor1Port);
        leftMotor2 = new PWMSparkMax(leftMotor2Port);
        rightMotor1 = new PWMSparkMax(rightMotor1Port);
        rightMotor2 = new PWMSparkMax(rightMotor2Port);
        myServo = new Servo(servoPort);
        joystick = new Joystick(joystickPort);
        CameraServer.startAutomaticCapture().setResolution(1280, 960);
        talonSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        talonSRX.setSelectedSensorPosition(0, 0, 10);
        // Creates UsbCamera and MjpegServer [1] and connects them
        CameraServer.startAutomaticCapture(0).setFPS(60);
        // guiInstance = new AprilTagClientGUI();
        lastError = 0;
        lastTimeStamp = 0;
        java camera = new PhotonCamera("photonvision");
        drive = new DifferentialDrive(leftMotor1, rightMotor1)
        
        krakenMotor = new TalonFX(11);
        TalonFXConfiguration config = new TalonFXConfiguration();

        m_chooser.setDefaultOption("Left Auto", kLeftAuto);
        m_chooser.addOption("Right Auto", kRightAuto);
        SmartDashboard.putData("Auto choices", m_chooser);
        SmartDashboard.putNumber("Shoot speed", shootSpeed);

        krakenMotor.getConfigurator().apply(config);
    }
    /** This function is run once each time the robot enters autonomous mode. */
    @Override
    public void autonomousInit() {
        // myServo.set(0.22);
        setpoint = -0.335;
        m_autoSelected = m_chooser.getSelected();
        System.out.println("Auto selected: " + m_autoSelected);
        m_timer.reset();
        m_timer.start();
    }
    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        autoTracking();
    }
    @Override
    public void teleopInit() {
        m_timer.reset();
        m_timer.start();
        setpoint = -0.07;
    }
    // クラス内のグローバル変数として定義
    double kP = 2.5; // Pゲイン
    double kD = 0.5; // Dゲイン
    double kI = 1;
    double iLimit = 0;
    double errorSum = 0;
    double setpoint; // 目標位置
    double lastError = 0; // 前回の偏差
    double lastTimeStamp = 0; // 前回のタイムスタンプ
    @Override
    public void teleopPeriodic() {
        boolean key1 = joystick.getRawButton(1);
        boolean key2 = joystick.getRawButton(2);
        // boolean key3 = joystick.getRawButton(3);
        // boolean key4 = joystick.getRawButton(4);
        boolean key5 = joystick.getRawButton(5);
        boolean key6 = joystick.getRawButton(6);
        boolean key7 = joystick.getRawButton(7);
        boolean key8 = joystick.getRawButton(8);
        boolean key9 = joystick.getRawButton(9);
        boolean key10 = joystick.getRawButton(10);
        if (key5) {
            // shoot
            rightNeoMotor.set(-1 * shootSpeed);
            leftNeoMotor.set(shootSpeed);
        } else if (key6) {
            rightNeoMotor.set(0.3);
            leftNeoMotor.set(-0.3);
        } else {
            rightNeoMotor.set(0);
            leftNeoMotor.set(0);
        }
        if (key5 && key6) {
            myServo.set(0.12);
        } else {
            myServo.set(0.364);
        }
        if (key2) {
            myServo.set(0.12);
        }
        // set the shooter speed
        if (key7) {
            shootSpeed = 0.3;
        } else if (key8) {
            shootSpeed = 1;
        }
        // ボタンによって目標位置を変える
        if (key9) {
            setpoint = -0.29; 
        } else if (key10) {
            setpoint = -0.19;
        }
        if (joystick.getRawAxis(2) > 0.1) {
            setpoint = -0.07;
        } else if (joystick.getRawAxis(3) > 0.1) {
            setpoint = -0.07;
        }
        // move driveBase
        double left = 1 * joystick.getRawAxis(1);         
        double right = -1 * joystick.getRawAxis(5); // ジョイスティックのY軸
        double rightSpeed = right;
        double leftSpeed = left;
        // leftMotor1.set(limitSpeed(rightSpeed));
        // leftMotor2.set(limitSpeed(rightSpeed));
        // rightMotor1.set(limitSpeed(leftSpeed));
        // rightMotor2.set(limitSpeed(leftSpeed));
        // teleopPeriodic内や任意の周期的に呼び出されるメソッド内
        double sensorPosition = talonSRX.getSelectedSensorPosition() * kDriveTick2Feet; // 現在位置の読み取り
        double error = setpoint - sensorPosition; // 偏差の計算
        // 現在の時間を取得し、前回からの経過時間を計算
        double currentTime = m_timer.get();
        double dt = currentTime - lastTimeStamp;
        if (Math.abs(error) < iLimit) {
            errorSum += error * dt;
        }
        // 変化率（D成分）の計算
        double errorRate = dt > 0 ? (error - lastError) / dt : 0;
        // 出力の計算
        // double outputSpeed = kP * error + kD * errorRate;
        double outputSpeed = kP * error + kI * errorSum + kD * errorRate;
        // モーター出力の制限 (-1.0 から 1.0 の範囲内)
        outputSpeed = Math.max(-1.0, Math.min(outputSpeed, 1.0));
        // モーターを動かす
        talonSRX.set(ControlMode.PercentOutput, outputSpeed);
        // スマートダッシュボードへの値の表示（デバッグ用）
        SmartDashboard.putNumber("Output Speed", outputSpeed);
        SmartDashboard.putNumber("Encoder Value", sensorPosition);
        SmartDashboard.putNumber("set Point", setpoint);
        SmartDashboard.putNumber(" Error", error);
        SmartDashboard.putNumber("errorRate", errorRate);
        SmartDashboard.putNumber("dt", dt);
        // SmartDashboard.putNumber("ID", guiInstance.getId());
        // SmartDashboard.putNumber("Distance Tag", guiInstance.getDis());
        // SmartDashboard.putNumber("roll", guiInstance.getRoll());
        // SmartDashboard.putNumber("pitch", guiInstance.getPitch());
        // SmartDashboard.putNumber("yaw", guiInstance.getYaw());
        // 前回の値の更新
        lastError = error;
        lastTimeStamp = currentTime;

        
       if (key1) {
         krakenMotor.setControl(new DutyCycleOut(0.5));
       }else{
         krakenMotor.setControl(new DutyCycleOut(0));
       }
       

    }
    private double limitSpeed(double speed) {
        return Math.max(-0.5, Math.min(0.5, speed));
    }
    /** This function is called once each time the robot enters test mode. */
    @Override
    public void testInit() {}
    @Override
    public void testPeriodic() {
    }
    public interface TagDataListener {
        void onDataReceived(double distance, double roll, double pitch, double yaw);
    }

    private void autoTracking() {
        PhotonPipelineResult result = camera.getLatestResult();
        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            double yawError = target.getYaw();
            double rotationSpeed = kP * yawError
            double forwardSpeed = 0.3;
            drive.arcadeDrive(forwardSpeed, rotationSpeed);
        }
        else {
            drive.arcadeDrive(0, 0.2)
        }
    }
}