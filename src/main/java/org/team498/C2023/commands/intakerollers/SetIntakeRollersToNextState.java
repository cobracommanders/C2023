package org.team498.C2023.commands.intakerollers;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team498.C2023.subsystems.IntakeRollers;

public class SetIntakeRollersToNextState extends CommandBase {
    private final IntakeRollers intake = IntakeRollers.getInstance();

    public SetIntakeRollersToNextState() {
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setToNextState();
    }
}
