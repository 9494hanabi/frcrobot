package frc.robot.commands;

import frc.robot.subsystems.Elevatorsub;
import frc.robot.subsystems.Goal;
import edu.wpi.first.wpilibj.Joystick;

import java.lang.annotation.Target;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climbsub;

public class Elevatorcom extends Command {
    private final Elevatorsub elevator;
    private final Joystick joystick;
    private final Goal goal;
    private final int upButton;
    private final int downButton;
    private final Climbsub climbsub;
    // private final int rightButton;
    // private final int leftButton;
    private final int modeChange;
    private double TargetPosition = 0;
    private boolean isDown = false;
    private int prevPOV = -1;
    private boolean isClimb = false;
    private int currentIndex = 2;
    private final String[] modes = {"Climb Mode", "Collect Mode", "L4 Mode", "L3 Mode", "L2 Mode", "L1 Mode"};
    private final int[] heights = {20, 30, 78, 55, 25, 3};
    double matchTime = DriverStation.getMatchTime();
    private int sequenceState = 0; // 追加: ステート管理
    private boolean sequenceRunning = false; // 追加: シーケンスの実行中フラグ
    private boolean climbRunning = false;

    public int getcurrentIndex() {
        return this.currentIndex;
    }

    public String[] getmodes() {
        return this.modes;
    }

    public Elevatorcom(Elevatorsub subsystem, Goal goal, Climbsub climbsub, Joystick joystick, int upButton, int downButton, int modeChange) {
    // public Elevetorcom(Elevetorsub subsystem, Joystick joysitck, int rihgtButton, int leftButton, int modeCange) {
        this.elevator = subsystem;
        this.goal = goal;
        this.climbsub = climbsub;
        this.joystick = joystick;
        this.upButton = upButton;
        this.downButton = downButton;
        this.modeChange = modeChange;
        addRequirements(elevator);
    }
    

    @Override
    public void execute() {
        System.out.println("Mode is " + modes[currentIndex]);
        int currentPOV = joystick.getPOV();

        // POVが変化したときのみ処理
        if (currentPOV != prevPOV) {
            if (currentPOV == 180) { // ↓が押されたとき
                if (currentIndex < modes.length - 1) {
                    currentIndex++;
                }
            } else if (currentPOV == 0) { // ↑が押されたとき
                if (currentIndex > 0) {
                    currentIndex--;
                }
            }
        }
    
        prevPOV = currentPOV; // 現在のPOVを保存
        

        // if (elevator.getElevatorHeightLeft() > TargetPosition) {
        //     elevator.moveDown(TargetPosition);
        // } else {
        //     elevator.setElevatorPosition(TargetPosition);
        // }

        // if (joystick.getRawButton(2)) {
        //     TargetPosition = 20;
        //     elevator.setElevatorPosition(TargetPosition);
        // }
        // else {
        //     TargetPosition = 0;
        //     elevator.climbDown(5);
        // }

        // if (sequenceRunning) {
        //     runSequence();
        // } else if (joystick.getRawButtonPressed(6) || joystick.getRawButton(5)) {
        //     if (currentIndex != 1 && currentIndex != 0) {
        //         sequenceRunning = true;
        //         sequenceState = 0;
        //     }
        // }

        // if (joystick.getRawButton(6)) {
        //     goal.center();
        // }
        // else {
        //     goal.goal();
        // }

        if (currentIndex == 0) {
            if (joystick.getRawButton(4)) {
                TargetPosition = heights[currentIndex];
                elevator.setElevatorPosition(TargetPosition);
            }
            if (joystick.getRawButton(1)) {
                elevator.moveDown(2);
            }
            if (joystick.getRawButton(5)) {
                elevator.climbDown(5);
            }
            if (joystick.getRawButton(6)) {
                climbsub.pull();
            }
            else{

            }
        }

        if (currentIndex == 1) {
            if (joystick.getRawButton(4)) {
                TargetPosition = selectPosition();
                elevator.setElevatorPosition(TargetPosition);
            }
            if (joystick.getRawButton(1)) {
                elevator.moveDown(3);
            }
        }

        if (currentIndex >= 2 && currentIndex <= 5) {
            if (joystick.getRawButton(4)) {
                TargetPosition = heights[currentIndex];
                elevator.setElevatorPosition(TargetPosition);
            }
            if (joystick.getRawButton(1)) {
                elevator.moveDown(5);
            }
        }


        // System.out.println("Target Position" + TargetPosition);
        // System.out.println("ElevatorHeight L" + elevator.getElevatorHeightLeft());
        // System.out.println("ElevatorHeight R" + elevator.getElevatorHeightRight());
    }

    private int selectPosition() {
        return heights[currentIndex];
    }

    private void runSequence() {
        // switch (sequenceState) {
        //     case 0:
        //         if (joystick.getRawButton(6)) {
        //             goal.center();
        //         }
        //         sequenceState++;
        //         break;
            // case 1:
            //     if (goal.isCentered()) {
            //         TargetPosition = selectPosition();
            //         elevator.setElevatorPosition(TargetPosition);
            //         sequenceState++;
            //     }
            //     break;
            // case 2:
            //     if (elevator.isAtPosition(TargetPosition)) {
            //         goal.right();
            //         sequenceState++;
            //     }
            //     break;
            // case 3:
            //     if (goal.isRight()) {
            //         goal.center();
            //         sequenceState++;
            //     }
            //     break;
            // case 4:
            //     if (goal.isCentered()) {
            //         TargetPosition = 3;
            //         elevator.moveDown(TargetPosition);
            //         sequenceState++;
            //     }
            //     break;
            // case 5:
            //     if (elevator.isAtPosition(TargetPosition)) {
            //         sequenceRunning = false;
            //     }
            //     break;
        // }
    }

    @Override
    public boolean isFinished() {
        return false; // 常に動作し続ける
    }

    public void resetTarget() {
        TargetPosition = 0.0;
    }
}

