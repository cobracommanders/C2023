package org.team498.C2023.commands.manipulator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team498.C2023.subsystems.Manipulator;

public class SetManipulatorState extends CommandBase {
    private final Manipulator manipulator = Manipulator.getInstance();
    private final Manipulator.State state;
    public SetManipulatorState(Manipulator.State state) {
        this.state = state;
    }

    @Override
    public void initialize() {
        manipulator.setState(state);
    }
}
