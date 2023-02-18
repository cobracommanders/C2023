package org.team498.C2023.commands.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team498.C2023.RobotState;
import org.team498.C2023.subsystems.Wrist;

public class RotateToPassPosition extends CommandBase {
    private final Wrist wrist = Wrist.getInstance();

    @Override
    public void initialize() {
        if (RobotState.getInstance().hasCone()) {
            wrist.setState(Wrist.State.PASS_CONE);
        } else {
            wrist.setState(Wrist.State.PASS_CUBE);
        }
    }

    @Override
    public boolean isFinished() {
        return wrist.atSetpoint();
    }
}

