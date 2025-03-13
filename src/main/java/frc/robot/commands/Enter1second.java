package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.Swerve;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

public class Enter1second extends Command {
    private final Timer timer = new Timer();
    Swerve swerve = new Swerve(0);

    public Enter1second(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        // ロボットの正面（前進）方向に一定速度で進む
        // ここでは0.3 m/sの速度で前進しています。必要に応じて調整してください。
        swerve.drive(new ChassisSpeeds(Swerve.maxLinearVelocityMetersPerSec, 0, 0.0), null);
    }

    @Override
    public void end(boolean interrupted) {
        // コマンド終了時、ロボットを停止する
        swerve.drive(new ChassisSpeeds(0.0, 0.0, 0.0), null);
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        // 1秒経過したらコマンド終了
        return timer.hasElapsed(1.0);
    }

    public static void Enter1second(Swerve swerve2) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'Enter1second'");
    }
}
