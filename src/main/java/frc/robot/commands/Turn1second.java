package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.Swerve;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

public class Turn1second extends Command {
    private final Swerve swerve;
    private final Timer timer = new Timer();

    public Turn1second(Swerve swerve) {
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
        // その場で回転させる。ここでは1.0 rad/sの回転速度を指定しています（必要に応じて調整してください）。
        swerve.drive(new ChassisSpeeds(0.0, 0.0, 1.0), null);
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
}
