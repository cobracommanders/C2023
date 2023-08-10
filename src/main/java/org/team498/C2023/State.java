package org.team498.C2023;

public enum State {
    IDLE(Flywheel.IDLE),
    LOW(Flywheel.LOW),
    HIGH(Flywheel.HIGH);

    public final Flywheel flywheel;

    State(Flywheel flywheel) {
        this.flywheel = flywheel;
    }

    public enum Flywheel {
        IDLE(0),
        LOW(500),
        HIGH(1000);

        public final double setpoint;

        Flywheel(double setpoint) {
            this.setpoint = setpoint;
        }
    }
}