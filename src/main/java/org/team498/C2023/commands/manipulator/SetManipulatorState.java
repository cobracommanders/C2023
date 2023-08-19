package org.team498.C2023.commands.manipulator;

import org.team498.C2023.State;
import org.team498.C2023.subsystems.Manipulator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetManipulatorState extends InstantCommand{
    private final State.Manipulator state;
    public SetManipulatorState(State.Manipulator state){
        addRequirements(Manipulator.getInstance());
        this.state = state;
    }

    @Override
    public void initialize() {
        Manipulator.getInstance().setState(state);
    }
}
