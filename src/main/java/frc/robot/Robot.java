package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.cameraserver.CameraServer;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;

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
    // エンコーダー関連の定数
    private final double kDriveTick2Feet = 1.0 / 4096 * 6 * Math.PI / 12;

    private static final String kLeftAuto = "leftSide";
    private static final String kRightAuto = "rightSide";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    // PID制御用の変数
    double kP = 2.5; // Pゲイン（※autoTrackingでは調整が必要かもしれない）
    double kD = 0.5; // Dゲイン
    double kI = 1;
    double iLimit = 0;
    double errorSum = 0;
    double setpoint;      // 目標位置
    double lastError = 0; // 前回の偏差
    double lastTimeStamp = 0; // 前回のタイムスタンプ

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
        CameraServer.startAutomaticCapture(0).setFPS(60);
        lastError = 0;
        lastTimeStamp = 0;
        
        // PhotonCameraとDifferentialDriveの初期化
        camera = new PhotonCamera("photonvision");
        drive = new DifferentialDrive(leftMotor1, rightMotor1);

        krakenMotor = new TalonFX(11);
        TalonFXConfiguration config = new TalonFXConfiguration();

        m_chooser.setDefaultOption("Left Auto", kLeftAuto);
        m_chooser.addOption("Right Auto", kRightAuto);
        SmartDashboard.putData("Auto choices", m_chooser);
        SmartDashboard.putNumber("Shoot speed", shootSpeed);

        krakenMotor.getConfigurator().apply(config);
    }

    /** 自動モード開始時の処理 */
    @Override
    public void autonomousInit() {
        setpoint = -0.335;
        m_autoSelected = m_chooser.getSelected();
        System.out.println("Auto selected: " + m_autoSelected);
        m_timer.reset();
        m_timer.start();
    }

    /** 自動モード周期処理 */
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

    @Override
    public void teleopPeriodic() {
        boolean key1 = joystick.getRawButton(1);
        boolean key2 = joystick.getRawButton(2);
        boolean key5 = joystick.getRawButton(5);
        boolean key6 = joystick.getRawButton(6);
        boolean key7 = joystick.getRawButton(7);
        boolean key8 = joystick.getRawButton(8);
        boolean key9 = joystick.getRawButton(9);
        boolean key10 = joystick.getRawButton(10);

        if (key5) {
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
        // シューター速度の設定
        if (key7) {
            shootSpeed = 0.3;
        } else if (key8) {
            shootSpeed = 1;
        }
        // ボタンにより目標位置を変更
        if (key9) {
            setpoint = -0.29; 
        } else if (key10) {
            setpoint = -0.19;
        }
        if (joystick.getRawAxis(2) > 0.1 || joystick.getRawAxis(3) > 0.1) {
            setpoint = -0.07;
        }

        double sensorPosition = talonSRX.getSelectedSensorPosition() * kDriveTick2Feet;
        double error = setpoint - sensorPosition;
        double currentTime = m_timer.get();
        double dt = currentTime - lastTimeStamp;
        if (Math.abs(error) < iLimit) {
            errorSum += error * dt;
        }
        double errorRate = dt > 0 ? (error - lastError) / dt : 0;
        double outputSpeed = kP * error + kI * errorSum + kD * errorRate;
        outputSpeed = Math.max(-1.0, Math.min(outputSpeed, 1.0));
        talonSRX.set(ControlMode.PercentOutput, outputSpeed);

        SmartDashboard.putNumber("Output Speed", outputSpeed);
        SmartDashboard.putNumber("Encoder Value", sensorPosition);
        SmartDashboard.putNumber("set Point", setpoint);
        SmartDashboard.putNumber(" Error", error);
        SmartDashboard.putNumber("errorRate", errorRate);
        SmartDashboard.putNumber("dt", dt);

        if (key1) {
            krakenMotor.setControl(new DutyCycleOut(0.5));
        } else {
            krakenMotor.setControl(new DutyCycleOut(0));
        }
    }

    private double limitSpeed(double speed) {
        return Math.max(-0.5, Math.min(0.5, speed));
    }

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}

    public interface TagDataListener {
        void onDataReceived(double distance, double roll, double pitch, double yaw);
    }

    // オートモード用AprilTag追従関数
    private void autoTracking() {
        PhotonPipelineResult result = camera.getLatestResult();
        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            double yawError = target.getYaw();
            double rotationSpeed = kP * yawError; // 調整が必要ならここを変更
            double forwardSpeed = 0.3;
            drive.arcadeDrive(forwardSpeed, rotationSpeed);
        } else {
            drive.arcadeDrive(0, 0.2);
        }
    }
}
