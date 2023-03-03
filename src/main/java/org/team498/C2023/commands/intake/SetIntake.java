package org.team498.C2023.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import org.team498.C2023.subsystems.Intake;

public class SetIntake extends InstantCommand {
    private final Intake intake = Intake.getInstance();
    private final Intake.State state;

    public SetIntake(Intake.State state) {
        this.state = state;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setState(state);
    }
}
