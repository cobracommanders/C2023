package org.team498.C2023;

public enum State {
    IDLE(Elevator.IDLE),
    LOW_CUBE(Elevator.LOW_CUBE),
    MID_CUBE(Elevator.MID_CUBE),
    HIGH_CUBE(Elevator.HIGH_CUBE);

    public final Elevator elevator;

    State(Elevator elevator) {
        this.elevator = elevator;
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

    public enum Elevator {
        IDLE(0.0),
        LOW_CUBE(0.0),
        MID_CUBE(0.0),
        HIGH_CUBE(0.0);

        public final double height;

        Elevator(double height) {
            this.height = height;
        }
    }
}