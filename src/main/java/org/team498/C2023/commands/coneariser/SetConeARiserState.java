package org.team498.C2023.commands.coneariser;

import org.team498.C2023.subsystems.ConeARiser;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetConeARiserState extends InstantCommand {
    private final ConeARiser.State state;
    private final ConeARiser coneARiser = ConeARiser.getInstance();
    public SetConeARiserState(ConeARiser.State state) {
        this.state = state;
    }

    @Override
    public void initialize() {
        coneARiser.setState(state);
    }
}
