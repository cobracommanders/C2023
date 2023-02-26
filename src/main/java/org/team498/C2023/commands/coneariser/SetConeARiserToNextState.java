package org.team498.C2023.commands.coneariser;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import org.team498.C2023.subsystems.ConeARiser;

public class SetConeARiserToNextState extends InstantCommand {
    private final ConeARiser coneARiser = ConeARiser.getInstance();
    public SetConeARiserToNextState() {
        addRequirements(coneARiser);
    }

    @Override
    public void initialize() {
        coneARiser.setToNextState();
    }
}
