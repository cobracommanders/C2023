package org.team498.C2023.commands.coneariser;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team498.C2023.RobotState;
import org.team498.C2023.subsystems.ConeARiser;

public class ConeARise extends CommandBase {
    private final ConeARiser coneARiser = ConeARiser.getInstance();

    @Override
    public void initialize() {
        if (RobotState.getInstance().inConeMode()) {
            coneARiser.setState(ConeARiser.State.COLLECT);
        } else {
            coneARiser.setState(ConeARiser.State.IDLE);
        }
    }
}
