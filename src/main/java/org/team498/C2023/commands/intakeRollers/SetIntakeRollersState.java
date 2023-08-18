package org.team498.C2023.commands.intakeRollers;

import org.team498.C2023.State;
import org.team498.C2023.subsystems.IntakeRollers;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetIntakeRollersState extends CommandBase{
    private final State.IntakeRollers state;

    public SetIntakeRollersState(State.IntakeRollers state){
        addRequirements(IntakeRollers.getInstance());
        this.state = state;
    }

    @Override
    public void initialize() {
        IntakeRollers.getInstance().setState(state);
    }
}
