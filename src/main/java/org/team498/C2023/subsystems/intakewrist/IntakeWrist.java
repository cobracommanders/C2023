package org.team498.C2023.subsystems.intakewrist;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;
import org.team498.C2023.Constants;
import org.team498.C2023.State;
import org.team498.C2023.subsystems.intakewrist.IntakeWristIOInputsAutoLogged;

public class IntakeWrist extends SubsystemBase {
    private final IntakeWristIO IO;
    private final IntakeWristIOInputsAutoLogged inputs = new IntakeWristIOInputsAutoLogged();

    private IntakeWrist() {
        IO = switch (Constants.mode) {
            case REAL, REPLAY, PRACTICE -> new IntakeWristIONEO();
            case SIM -> new IntakeWristIO() {};
        };
    }

    @Override
    public void periodic() {
        IO.updateInputs(inputs);
        Logger.getInstance().processInputs("IntakeWrist", inputs);
        IO.setBrakeMode(RobotState.isEnabled());

        SmartDashboard.putData(this);
        SmartDashboard.putBoolean("IntakeWrist at Setpoint", atSetpoint());
        SmartDashboard.putNumber("IntakeWrist Error", inputs.targetAngle - inputs.angle);
        SmartDashboard.putNumber("IntakeWrist Angle", inputs.angle);
    }

    public boolean atSetpoint() {
        return Math.abs(inputs.targetAngle - inputs.angle) < 0.05;
    }

    public void setState(State.IntakeWrist state) {
        IO.setPosition(state.position);
        Logger.getInstance().recordOutput("IntakeWrist State", state.name());
    }


    private static IntakeWrist instance;

    public static IntakeWrist getInstance() {
        if (instance == null) {
            instance = new IntakeWrist();
        }
        return instance;
    }
}
