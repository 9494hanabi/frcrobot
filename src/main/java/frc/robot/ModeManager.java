package frc.robot;

public class ModeManager {
    private Mode currentMode = Mode.L4; // 初期モード

    public Mode getCurrentMode() {
        return currentMode;
    }

    public void moveUp() {
        currentMode = currentMode.up();
    }

    public void moveDown() {
        currentMode = currentMode.down();
    }

    public void reset() {
        currentMode = Mode.L4;
    }
}
