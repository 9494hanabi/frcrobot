package frc.robot.commands;

import frc.robot.subsystems.Elevatorsub;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public class Elevatorcom extends Command {
    private final Elevatorsub elevator;
    private final Joystick joystick;
    private final int upButton;
    private final int downButton;
    private double TargetPosition = 0;
    private boolean isDown = false;
    private boolean prevUpButtonState = false;
    private boolean prevDownButtonState = false;
    private int currentIndex = 0;
    private final double[] heights = {0, 2, 3, 4, 6, 8};
    double matchTime = DriverStation.getMatchTime();

    public Elevatorcom(Elevatorsub subsystem, Joystick joystick, int upButton, int downButton) {
        this.elevator = subsystem;
        this.joystick = joystick;
        this.upButton = upButton;
        this.downButton = downButton;
        addRequirements(elevator);
    }

    

    @Override
    public void execute() {
        boolean upButtonState = joystick.getRawButton(100);  // UP button elevator
        boolean downButtonState = joystick.getRawButton(100);  // DOWN button elevator

        // ボタンが押された瞬間（false -> true）のみ処理
        if (upButtonState && !prevUpButtonState) {
            // インデックスの範囲内かチェックし、1段階上げる
            if (currentIndex < heights.length - 1) {
                currentIndex++;
            }
        }
        if (downButtonState && !prevDownButtonState) {
            // インデックスの範囲内かチェックし、1段階上げる
            if (currentIndex > 0) {
                currentIndex--;
            }
        }
        // 前回状態を更新
        prevUpButtonState = upButtonState;
        prevDownButtonState = downButtonState;
        
        if (joystick.getRawButton(8)) {
            elevator.climbElevator();
        }

        // if (matchTime <= 15.0 && matchTime > 0) {
        //     if (joystick.getRawButton(8)) {
        //         elevator.climbElevator();
        //     } else {
        //         elevator.setelev(3);
        //     }
        // } else {
        //     if (elevator.getElevatorHeightR() > TargetPosition) {
        //         elevator.downelev(TargetPosition);
        //         System.out.println("isDown is working!");
        //     } else {
        //         elevator.setelev(heights[currentIndex]);
        //     }
        // }

        // System.out.println("Target Position" + TargetPosition);
        // System.out.println("ElevatorHeight L" + elevator.getElevatorHeightL());
        // System.out.println("ElevatorHeight R" + elevator.getElevatorHeightR());
    }

    @Override
    public boolean isFinished() {
        return false; // 常に動作し続ける
    }

    public void resetTarget() {
        TargetPosition = 0.0;
    }
}

