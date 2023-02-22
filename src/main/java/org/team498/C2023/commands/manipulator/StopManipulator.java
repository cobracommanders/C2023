package org.team498.C2023.commands.manipulator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.team498.C2023.subsystems.Manipulator;

public class StopManipulator extends InstantCommand {
    private final Manipulator manipulator = Manipulator.getInstance();

    @Override
    public void initialize() {
        manipulator.setState(Manipulator.State.IDLE);
    }
}
