package org.team498.C2023.commands.coneariser;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team498.C2023.subsystems.ConeARiser;

public class PassFromConeARiser extends CommandBase {
    private final ConeARiser coneARiser = ConeARiser.getInstance();

    @Override
    public void initialize() {
        coneARiser.setState(ConeARiser.State.REJECT);
    }
}
