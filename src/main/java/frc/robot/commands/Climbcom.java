package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climbsub;
import frc.robot.Mode;
import frc.robot.ModeManager;

public class Climbcom extends Command {
    private final Climbsub climbsub;
    private final ModeManager modeManager;
    private final Runnable action;

    public Climbcom(Climbsub climbsub, ModeManager modeManager, Runnable action) {
        this.climbsub = climbsub;
        this.modeManager = modeManager;
        this.action = action;
        addRequirements(climbsub);
    }

    @Override
    public void execute() {
        if (modeManager.getCurrentMode() == Mode.CLIMB) {
            action.run(); // Mode.CLIMBのときだけ動作
        } else {
            climbsub.dontpull(); // セーフティ
        }
    }

    @Override
    public void end(boolean interrupted) {
        climbsub.dontpull(); // 安全確保
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
