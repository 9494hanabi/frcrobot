package frc.robot.commands;

import frc.robot.subsystems.Elevatorsub;
import frc.robot.ModeManager;
import frc.robot.Mode;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public class Elevatorcom extends Command {
    private final Elevatorsub elevator;
    private final ModeManager modeManager;
    private final BooleanSupplier moveUpButton;
    private final BooleanSupplier goToL1Supplier;
    private final BooleanSupplier climbMidSupplier;
    private final BooleanSupplier climbDownSupplier;
    private double TargetPosition = 0;
    private boolean prevMoveUpPressed = false;
    double matchTime = DriverStation.getMatchTime();

    // public String[] getmodes() {
    //     return this.modes;
    // }

    public Elevatorcom(Elevatorsub subsystem, ModeManager modeManager, BooleanSupplier moveUpButton, BooleanSupplier goToL1Supplier, BooleanSupplier climbMidSupplier, BooleanSupplier climbDownSupplier) {
        this.elevator = subsystem;
        this.modeManager = modeManager;
        this.moveUpButton = moveUpButton;
        this.goToL1Supplier = goToL1Supplier;
        this.climbMidSupplier = climbMidSupplier;
        this.climbDownSupplier = climbDownSupplier;
        addRequirements(elevator);
    }
    

    @Override
    public void execute() {
        boolean nowPressed = moveUpButton.getAsBoolean();
        System.out.println("height is " + elevator.getElevatorHeightLeft());
        Mode currentMode = modeManager.getCurrentMode();
        System.out.println("Mode is " + currentMode.getLabel());

        if (nowPressed && !prevMoveUpPressed) {
            // 押された瞬間だけTargetPositionを更新
            TargetPosition = currentMode.getTargetHeight();
            System.out.println("New TargetPosition set: " + TargetPosition);
        }
    

        if (TargetPosition <= elevator.getElevatorHeightLeft()) {
            elevator.moveDown(TargetPosition);
        }
        else {   
        elevator.setElevatorPosition(TargetPosition);
        }

        if (goToL1Supplier.getAsBoolean()) {
            TargetPosition = Mode.L1.getTargetHeight();
        }
        



        if (currentMode == Mode.CLIMB) {
            if (climbMidSupplier.getAsBoolean()) {
                TargetPosition = 25;
            }
            if (climbDownSupplier.getAsBoolean()) {
                elevator.climbDown(5);
            }
        }

        if (currentMode == Mode.COLLECT) {
        }

        if (currentMode == Mode.L4 || currentMode == Mode.L3 || currentMode == Mode.L2 || currentMode == Mode.L1) {
        }
    }

    @Override
    public boolean isFinished() {
        return false; // 常に動作し続ける
    }

    public void resetTarget() {
        TargetPosition = 0.0;
    }
}

