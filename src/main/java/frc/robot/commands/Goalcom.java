package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Goal;
import frc.robot.ModeManager;
import frc.robot.Mode;
import java.util.function.BooleanSupplier;

public class Goalcom extends Command {
    private final Goal goal;
    private final ModeManager modeManager;
    private final BooleanSupplier isLeftSupplier;
    private final BooleanSupplier isRightSupplier;
    private final BooleanSupplier isShootSupplier;

    public Goalcom(
        Goal goal,
        ModeManager modeManager,
        BooleanSupplier isLeftSupplier,
        BooleanSupplier isRightSupplier,
        BooleanSupplier isShootSupplier
    ) {
        this.goal = goal;
        this.modeManager = modeManager;
        this.isLeftSupplier = isLeftSupplier;
        this.isRightSupplier = isRightSupplier;
        this.isShootSupplier = isShootSupplier;
        addRequirements(goal);
    }

    @Override
    public void execute() {
        if (isScoreMode()) {
            boolean isLeft = isLeftSupplier.getAsBoolean();
            boolean isRight = isRightSupplier.getAsBoolean();
            boolean isShoot = isShootSupplier.getAsBoolean();

            goal.goal(isLeft, isRight, isShoot);
        }
    }

    private boolean isScoreMode() {
        Mode mode = modeManager.getCurrentMode();
        return mode == Mode.L4 || mode == Mode.L3 || mode == Mode.L2 || mode == Mode.L1;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
