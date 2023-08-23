package org.team498.C2023.commands.manipulator;

import org.team498.C2023.RobotState;
import org.team498.C2023.State;
import org.team498.C2023.subsystems.Manipulator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetManipulatorState extends InstantCommand{
    private State.Manipulator state;
    public SetManipulatorState(){
        addRequirements(Manipulator.getInstance());
    }

    @Override
    public void initialize() {
        state = RobotState.getInstance().getState().manipulator;
        Manipulator.getInstance().setState(state);
    }
}
