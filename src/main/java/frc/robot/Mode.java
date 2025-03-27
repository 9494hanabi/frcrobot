package frc.robot;

public enum Mode {
    CLIMB("Climb Mode", 40),
    COLLECT("Collect Mode", 30),
    L4("L4 Mode", 45),
    L3("L3 Mode", 45),
    L2("L2 Mode", 20),
    L1("L1 Mode", 3);

    private final String label;
    private final int targetHeight;

    Mode(String label, int height) {
        this.label = label;
        this.targetHeight = height;
    }

    public String getLabel() {
        return label;
    }

    public int getTargetHeight() {
        return targetHeight;
    }

    public Mode up() {
        int newIndex = Math.max(0, this.ordinal() - 1);
        return Mode.values()[newIndex];
    }

    public Mode down() {
        int newIndex = Math.min(Mode.values().length - 1, this.ordinal() + 1);
        return Mode.values()[newIndex];
    }
}
