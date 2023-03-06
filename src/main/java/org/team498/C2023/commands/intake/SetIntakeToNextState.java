package org.team498.C2023.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import org.team498.C2023.subsystems.Intake;

public class SetIntakeToNextState extends InstantCommand {
    private final Intake intake = Intake.getInstance();

    public SetIntakeToNextState() {
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setToNextState();
    }
}
