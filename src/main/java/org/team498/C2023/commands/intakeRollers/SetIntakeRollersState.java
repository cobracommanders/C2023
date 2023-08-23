package org.team498.C2023.commands.intakeRollers;

import org.team498.C2023.RobotState;
import org.team498.C2023.State;
import org.team498.C2023.subsystems.IntakeRollers;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetIntakeRollersState extends InstantCommand{
    private State.IntakeRollers state;

    public SetIntakeRollersState(){
        addRequirements(IntakeRollers.getInstance());
    }

    @Override
    public void initialize() {
        state = RobotState.getInstance().getState().intakeRollers;
        IntakeRollers.getInstance().setState(state);
    }
}
