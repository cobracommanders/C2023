package org.team498.C2023.subsystems.elevatorwrist;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;
import org.team498.C2023.Constants;
import org.team498.C2023.RobotPosition;
import org.team498.C2023.ShootTables;
import org.team498.C2023.State;

public class ElevatorWrist extends SubsystemBase {
    private final ElevatorWristIO IO;
    private final ElevatorWristIOInputsAutoLogged inputs = new ElevatorWristIOInputsAutoLogged();

    private ElevatorWrist() {
        IO = switch (Constants.mode) {
            case REAL, REPLAY, PRACTICE -> new ElevatorWristIONEO();
            case SIM -> new ElevatorWristIO() {};
        };
    }

    @Override
    public void periodic() {
        IO.updateInputs(inputs);
        Logger.getInstance().processInputs("ElevatorWrist", inputs);
        IO.setBrakeMode(RobotState.isEnabled());

        SmartDashboard.putData(this);
        SmartDashboard.putBoolean("ElevatorWrist at Setpoint", atSetpoint());
        SmartDashboard.putNumber("ElevatorWrist Error", inputs.targetAngle - inputs.angle);
        SmartDashboard.putNumber("ElevatorWrist Angle", inputs.angle);
    }

    public boolean atSetpoint() {
        return Math.abs(inputs.targetAngle - inputs.angle) < 0.02;
    }

    public void setManual(double speed) {
        IO.setManual(speed);
    }

    public void setState(State.ElevatorWrist state) {
        IO.setPosition(getSetpoint(state, RobotPosition.getFutureScoringNodeDistance()));
        Logger.getInstance().recordOutput("ElevatorWrist State", state.name());
    }

    private double getSetpoint(State.ElevatorWrist state, double interpolatedValue) {
        return switch (state) {
            case SHOOT_DRIVE_CUBE_MID -> ShootTables.midCube.wristAngle.getInterpolatedValue(interpolatedValue);
            case SHOOT_DRIVE_CUBE_TOP -> ShootTables.topCube.wristAngle.getInterpolatedValue(interpolatedValue);
            case SHOOT_DRIVE_CONE_MID -> ShootTables.midCone.wristAngle.getInterpolatedValue(interpolatedValue);
            default -> state.setpoint;
        };
    }

    private static ElevatorWrist instance;

    public static ElevatorWrist getInstance() {
        if (instance == null) {
            instance = new ElevatorWrist();
        }
        return instance;
    }
}
