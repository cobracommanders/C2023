package org.team498.C2023.subsystems.elevator;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;
import org.team498.C2023.Constants;
import org.team498.C2023.RobotPosition;
import org.team498.C2023.ShootTables;
import org.team498.C2023.State;

public class Elevator extends SubsystemBase {
    private final ElevatorIO IO;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private Elevator() {
        IO = switch (Constants.mode) {
            case REAL, REPLAY, PRACTICE -> new ElevatorIOFalcon500();
            case SIM -> new ElevatorIO() {};
        };
    }

    @Override
    public void periodic() {
        IO.updateInputs(inputs);
        Logger.getInstance().processInputs("Elevator", inputs);
        IO.setBrakeMode(RobotState.isEnabled());

        // SmartDashboard.putData(this);
        // SmartDashboard.putBoolean("Elevator at Setpoint", atSetpoint());
        // SmartDashboard.putNumber("Elevator Error", inputs.targetPositionMeters - inputs.positionMeters);
        // SmartDashboard.putNumber("Elevator Position", inputs.positionMeters);
    }

    public boolean atSetpoint() {
        return Math.abs(inputs.targetPositionMeters - inputs.positionMeters) < 0.05;
    }

    public void setManual(double speed) {
        IO.setManual(speed);
    }

    public void setState(State.Elevator state) {
        IO.setPosition(getSetpoint(state, RobotPosition.getFutureScoringNodeDistance()));
        Logger.getInstance().recordOutput("Elevator State", state.name());
    }

    public boolean aboveIntakeHeight() {
        return inputs.positionMeters > 0.4; //TODO find out the real number here
    }

    private double getSetpoint(State.Elevator state, double interpolatedValue) {
        return switch (state) {
            case SHOOT_DRIVE_CUBE_MID -> ShootTables.midCube.elevatorHeight.getInterpolatedValue(interpolatedValue);
            case SHOOT_DRIVE_CUBE_TOP -> ShootTables.topCube.elevatorHeight.getInterpolatedValue(interpolatedValue);
            case SHOOT_DRIVE_CONE_MID -> ShootTables.midCone.elevatorHeight.getInterpolatedValue(interpolatedValue);
            default -> state.setpoint;
        };
    }

    public void updateInitialPosition(boolean inAutoPose) {
        IO.updateInitialPosition(inAutoPose);
    }


    private static Elevator instance;

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }
        return instance;
    }
}
