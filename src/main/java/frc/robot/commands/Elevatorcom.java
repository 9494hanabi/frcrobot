package frc.robot.commands;

import frc.robot.subsystems.Elevatorsub;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;

public class Elevatorcom extends Command {
    private final Elevatorsub elevator;
    private final Joystick joystick;
    private final int upButton;
    private final int downButton;
    private double TargetPosition = 0.0;

    public Elevatorcom(Elevatorsub subsystem, Joystick joystick, int upButton, int downButton) {
        this.elevator = subsystem;
        this.joystick = joystick;
        this.upButton = upButton;
        this.downButton = downButton;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.setelev(TargetPosition);

        if (joystick.getRawButton(upButton)) {
            TargetPosition = 6;
        } else if (joystick.getRawButton(downButton)) {
            TargetPosition = 0;
        } 
        // else {
        //     elevator.stopElevator(TargetPosition); // 停止
        // }

        System.out.println("Target Position" + TargetPosition);
        System.out.println("ElevatorHeight L" + elevator.getElevatorHeightL());
        System.out.println("ElevatorHeight R" + elevator.getElevatorHeightR());
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stopElevator(TargetPosition);
    }

    @Override
    public boolean isFinished() {
        return false; // 常に動作し続ける
    }
}

