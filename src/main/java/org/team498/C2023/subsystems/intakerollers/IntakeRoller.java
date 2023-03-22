package org.team498.C2023.subsystems.intakerollers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;
import org.team498.C2023.Constants;
import org.team498.C2023.State;

public class IntakeRoller extends SubsystemBase {
    private final IntakeRollerIO IO;
    private final IntakeRollerIOInputsAutoLogged inputs = new IntakeRollerIOInputsAutoLogged();

    private IntakeRoller() {
        IO = switch (Constants.mode) {
            case REAL, REPLAY, PRACTICE -> new IntakeRollerIOFalcon500NEO();
            case SIM -> new IntakeRollerIO() {};
        };
    }

    @Override
    public void periodic() {
        IO.updateInputs(inputs);
        Logger.getInstance().processInputs("IntakeRoller", inputs);

        SmartDashboard.putData(this);
    }

    public void setState(State.IntakeRollers state) {
        IO.setState(state);
        Logger.getInstance().recordOutput("IntakeRoller State", state.name());
    }


    private static IntakeRoller instance;

    public static IntakeRoller getInstance() {
        if (instance == null) {
            instance = new IntakeRoller();
        }
        return instance;
    }
}
