package org.team498.C2023.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team498.C2023.*;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private final ElevatorFeedforward FF;
    private final TrapezoidProfile.Constraints constraints;

    private State.Elevator currentState = State.Elevator.IDLE;
    private ControlMode controlMode = ControlMode.MANUAL;

    private double speed = 0;
    private double setpoint = 0;

    private Elevator(ElevatorIO io) {
        this.io = io;


        switch (Constants.mode) {
            case REAL, REPLAY -> {
                io.configPID(5, 0, 0);
                constraints = new TrapezoidProfile.Constraints(10, 6);
            }
            case SIM -> {
                io.configPID(5, 0, 0);
                constraints = new TrapezoidProfile.Constraints(10, 6.0);
            }
            case PRACTICE -> {
                io.configPID(5, 0, 0);
                constraints = new TrapezoidProfile.Constraints(5, 3);
            }
            default -> {io.configPID(0, 0, 0); constraints = new TrapezoidProfile.Constraints(0, 0);}
        }

        FF = new ElevatorFeedforward(0, 0.075, 0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Elevator", inputs);

        setpoint = getSetpoint(currentState, RobotPositions.getFutureScoringNodeDistance());

        if (controlMode == ControlMode.PID) {
            var profile = new TrapezoidProfile(constraints, new TrapezoidProfile.State(setpoint, 0), new TrapezoidProfile.State(inputs.positionMeters, 0));
            io.setPosition(profile.calculate(Robot.DEFAULT_PERIOD).position, FF.calculate(inputs.velocityMetersPerSecond) / 12);
        } else {
            io.setSpeed(speed);
        }

        SmartDashboard.putData(this);
        SmartDashboard.putNumber("Elevator Position", inputs.positionMeters);
        SmartDashboard.putBoolean("Elevator at Setpoint", atSetpoint());
        SmartDashboard.putNumber("Elevator Error", setpoint - inputs.positionMeters);

        Logger.getInstance().recordOutput("ElevatorSetpointMeters", setpoint);
    }

    public void setState(State.Elevator state) {
        this.controlMode = ControlMode.PID;
        this.currentState = state;
    }

    public void setSpeed(double speed) {
        this.controlMode = ControlMode.MANUAL;
        this.speed = speed;
    }

    private double getSetpoint(State.Elevator state, double interpolatedValue) {
        return switch (state) {
            case SHOOT_DRIVE_CUBE_MID -> ShootTables.midCube.elevatorHeight.getInterpolatedValue(interpolatedValue);
            case SHOOT_DRIVE_CUBE_TOP -> ShootTables.topCube.elevatorHeight.getInterpolatedValue(interpolatedValue);
            case SHOOT_DRIVE_CONE_MID -> ShootTables.midCone.elevatorHeight.getInterpolatedValue(interpolatedValue);
            default -> state.setpoint;
        };
    }

    public boolean aboveIntakeHeight() {
        return inputs.positionMeters > 0.4; //TODO find out the real number here
    }

    public boolean atSetpoint() {
        return Math.abs(setpoint - inputs.positionMeters) < 0.025;
    }


    public void updateInitialPosition(boolean inAutoPose) {
        io.setInitialPosition(inAutoPose);
    }

    public enum ControlMode {
        PID,
        MANUAL
    }

    private static Elevator instance;

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator(switch (Constants.mode) {
                case REAL, REPLAY -> new ElevatorIOCompRobot();
                default -> new ElevatorIO() {};
            });
        }
        return instance;
    }
}
