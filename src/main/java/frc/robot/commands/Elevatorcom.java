package frc.robot.commands;

import frc.robot.subsystems.Elevatorsub;
import edu.wpi.first.wpilibj.Joystick;

import java.lang.annotation.Target;

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
    private boolean isClimb = false;
    private int currentIndex = 1;
    private final String[] modes = {"ClimbMode", "L4 Mode", "L3 Mode", "L2 Mode", "L1 Mode", "L1 Mode"};
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
        System.out.println("TargetPosition : " + TargetPosition);
        boolean upButtonState = joystick.getRawButton(4);  // UP button elevator
        boolean downButtonState = joystick.getRawButton(1);  // DOWN button elevator

        // ボタンが押された瞬間（false -> true）のみ処理
        if (upButtonState && !prevUpButtonState) {
            // インデックスの範囲内かチェックし、1段階上げる
            if (currentIndex < modes.length - 1) {
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


        // ２．うまくいけば３へ
        // if (joystick.getRawButton(4)) {
        //     isClimb = true;
        // } else if (joystick.getRawButton(1)) {
        //     isClimb = false;
        // }
        // if (isClimb) {
        //     elevator.climbElevator();
        // } else {
        //     // elevator.setelev(3);
        // }

        // ３．マッチタイムでクライムかそうではないかを調整
        // if (matchTime <= 15.0 && matchTime > 0) {
        //     if (joystick.getRawButton(8)) {
        //         elevator.climbElevator();
        //     } else {
        //         elevator.setelev(3);
        //     }
        // } else {
        //     if (elevator.getElevatorHeightR() > heights[currentIndex]) {
        //         elevator.downelev(heights[currentIndex]);
        //         System.out.println("isDown is working!");
        //     } else {
        //         elevator.setelev(heights[currentIndex]);
        //     }
        // }

        
        // １．うまく上がれば　２へ進む
        if (joystick.getRawButton(2)) {
            TargetPosition = 3;
        } else {
            TargetPosition = 0;
        }

        elevator.setElevatorPosition(TargetPosition);

        // System.out.println("Target Position" + TargetPosition);
        System.out.println("ElevatorHeight L" + elevator.getElevatorHeightLeft());
        System.out.println("ElevatorHeight R" + elevator.getElevatorHeightRight());
    }

    @Override
    public boolean isFinished() {
        return false; // 常に動作し続ける
    }

    public void resetTarget() {
        TargetPosition = 0.0;
    }
}

