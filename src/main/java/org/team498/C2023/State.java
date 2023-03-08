package org.team498.C2023;

public enum State {
    SPIT_CUBE(Elevator.SPIT, ElevatorWrist.SPIT, IntakeWrist.SPIT, IntakeRollers.SPIT, Manipulator.SPIT, ConeARiser.IDLE),

    TOP_CONE(Elevator.TOP_CONE, ElevatorWrist.TOP_CONE, IntakeWrist.IDLE_IN, IntakeRollers.IDLE, Manipulator.TOP_CONE, ConeARiser.IDLE),
    TOP_CUBE(Elevator.TOP_CUBE, ElevatorWrist.TOP_CUBE, IntakeWrist.IDLE_IN, IntakeRollers.IDLE, Manipulator.TOP_CUBE, ConeARiser.IDLE),

    MID_CONE(Elevator.MID_CONE, ElevatorWrist.MID_CONE, IntakeWrist.IDLE_IN, IntakeRollers.IDLE, Manipulator.MID_CONE, ConeARiser.IDLE),
    MID_CUBE(Elevator.MID_CUBE, ElevatorWrist.MID_CUBE, IntakeWrist.IDLE_IN, IntakeRollers.IDLE, Manipulator.MID_CUBE, ConeARiser.IDLE),

    IDLE_CONE(Elevator.IDLE, ElevatorWrist.IDLE_CONE, IntakeWrist.IDLE_IN, IntakeRollers.IDLE, Manipulator.IDLE, ConeARiser.IDLE),
    IDLE_CUBE(Elevator.IDLE, ElevatorWrist.IDLE_CUBE, IntakeWrist.IDLE_IN, IntakeRollers.IDLE, Manipulator.IDLE, ConeARiser.IDLE),

    OUTTAKE(Elevator.IDLE, ElevatorWrist.IDLE_CUBE, IntakeWrist.OUTTAKE, IntakeRollers.OUTTAKE, Manipulator.SPIT, ConeARiser.IDLE),

    GROUND_CUBE(Elevator.INTAKE, ElevatorWrist.INTAKE, IntakeWrist.INTAKE, IntakeRollers.INTAKE, Manipulator.INTAKE_CUBE, ConeARiser.IDLE),

    UNSTICK_CUBE(Elevator.UNSTICK_CUBE, ElevatorWrist.TRAVEL, IntakeWrist.TRAVEL_CUBE, IntakeRollers.IDLE, Manipulator.INTAKE_CUBE, ConeARiser.IDLE),

    CONEARISER_CUBE(Elevator.CONEARISER_CUBE, ElevatorWrist.CONEARISER_CUBE, IntakeWrist.IDLE_IN, IntakeRollers.IDLE, Manipulator.INTAKE_CUBE, ConeARiser.IDLE),

    DOUBLE_SS(Elevator.DOUBLE_SS, ElevatorWrist.DOUBLE_SS, IntakeWrist.IDLE_IN, IntakeRollers.IDLE, Manipulator.INTAKE_CONE, ConeARiser.IDLE),
    SINGLE_SS(Elevator.SINGLE_SS, ElevatorWrist.SINGLE_SS, IntakeWrist.IDLE_OUT, IntakeRollers.IDLE, Manipulator.INTAKE_CUBE, ConeARiser.IDLE),
    INTERPOLATE(Elevator.INTERPOLATE, ElevatorWrist.INTERPOLATE, IntakeWrist.IDLE_IN, IntakeRollers.IDLE, Manipulator.INTERPOLATE, ConeARiser.IDLE),

    TRAVEL_EMPTY(Elevator.IDLE, ElevatorWrist.TRAVEL, IntakeWrist.IDLE_IN, IntakeRollers.IDLE, Manipulator.IDLE, ConeARiser.IDLE),
    TRAVEL_CONE(Elevator.IDLE, ElevatorWrist.TRAVEL, IntakeWrist.TRAVEL_CONE, IntakeRollers.IDLE, Manipulator.INTAKE_CONE, ConeARiser.IDLE),
    TRAVEL_CUBE(Elevator.IDLE, ElevatorWrist.TRAVEL, IntakeWrist.TRAVEL_CUBE, IntakeRollers.IDLE, Manipulator.IDLE, ConeARiser.IDLE),
    ;

    public final Elevator elevator;
    public final ElevatorWrist elevatorWrist;
    public final IntakeWrist intakeWrist;
    public final IntakeRollers intakeRollers;
    public final Manipulator manipulator;
    public final ConeARiser coneARiser;

    State(Elevator elevator, ElevatorWrist elevatorWrist, IntakeWrist intakeWrist, IntakeRollers intakeRollers, Manipulator manipulator, ConeARiser coneARiser) {
        this.elevator = elevator;
        this.elevatorWrist = elevatorWrist;
        this.intakeWrist = intakeWrist;
        this.intakeRollers = intakeRollers;
        this.manipulator = manipulator;
        this.coneARiser = coneARiser;
    }

    public enum Elevator {
        CONEARISER_CONE(0),
        CONEARISER_CUBE(0),
        SINGLE_SS(0.16745),
        DOUBLE_SS(0.62),

        SPIT(0),

        MID_CONE(0.875),
        MID_CUBE(0.5),

        TOP_CONE(0.875),
        TOP_CUBE( 0.885),

        AUTO_SHOT(0),

        INTAKE(0),

        IDLE(0),

        UNSTICK_CUBE(0.4),

        INTERPOLATE(0);

        public final double setpoint;

        Elevator(double setpoint) {
            this.setpoint = setpoint;
        }
    }

    public enum ElevatorWrist {
        SINGLE_SS(0.06),
        DOUBLE_SS(0.109352),

        SPIT(-0.038142),

        MID_CONE(0.1),
        MID_CUBE(-0.04),

        TOP_CONE(0.25),
        TOP_CUBE(-0.02),

        TRAVEL(0.041067),

        INTAKE(-0.05),

        AUTO_SHOT(0),

        IDLE_CUBE(-0.06),
        IDLE_CONE(-0.02),

        CONEARISER_CUBE(-0.065),
        CONEARISER_CONE(0),

        INTERPOLATE(0);

        public final double setpoint;

        ElevatorWrist(double setpoint) {
            this.setpoint = setpoint;
        }
    }

    public enum IntakeRollers {
        INTAKE(0.5, 0.5),
        SPIT(0.45, -0.45),
        IDLE(0, 0),
        OUTTAKE(-0.5, -0.5);

        public final double bottomRollerSpeed;
        public final double topRollerSpeed;

        IntakeRollers(double bottomRollerSpeed, double topRollerSpeed) {
            this.bottomRollerSpeed = bottomRollerSpeed;
            this.topRollerSpeed = topRollerSpeed;
        }
    }

    public enum IntakeWrist {
        INTAKE(0.1),
        SPIT(0.35),
        IDLE_OUT(0),
        TRAVEL_CUBE(0.2),
        TRAVEL_CONE(0.2),
        IDLE_IN(0.4),
        OUTTAKE(0.1);

        public final double position;

        IntakeWrist(double position) {
            this.position = position;
        }
    }

    public enum Manipulator {
        INTAKE_CONE(1),
        INTAKE_CUBE(-1),

        SPIT(0.9),
        MID_CONE(-1),
        MID_CUBE(0.5),
        TOP_CONE(-1),
        TOP_CUBE(0.5),

        AUTO_SHOT(0.5),
        INTERPOLATE(0),
        HOLD(1),
        IDLE(0);

        public final double setpoint;

        Manipulator(double setpoint) {
            this.setpoint = setpoint;
        }
    }

    public enum ConeARiser {
        COLLECT(0.5, 0.5),
        REJECT(-1, -1),
        IDLE(0, 0);

        public final double frontBackSpeed;
        public final double leftRightSpeed;

        ConeARiser(double frontBackSpeed, double leftRightSpeed) {
            this.frontBackSpeed = frontBackSpeed;
            this.leftRightSpeed = leftRightSpeed;
        }
    }
}