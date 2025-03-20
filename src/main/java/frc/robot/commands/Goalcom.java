package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Goal;

public class Goalcom extends Command {
    private final Goal goalSubsystem;
    
    public Goalcom(Goal goalSubsystem) {
        this.goalSubsystem = goalSubsystem;
        addRequirements(goalSubsystem);
    }
    
    @Override
    public void execute() {
        // Goal サブシステムの goal() メソッドを呼び出す
        goalSubsystem.goal();
    }
    
    @Override
    public boolean isFinished() {
        // 常時実行する場合は false を返す（必要に応じて条件を追加してください）
        return false;
    }
}
