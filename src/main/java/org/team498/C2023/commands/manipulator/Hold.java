package org.team498.C2023.commands.manipulator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team498.C2023.subsystems.Manipulator;

public class Hold extends CommandBase {
    private final Manipulator manipulator = Manipulator.getInstance();

    public Hold() {
        addRequirements(manipulator);
    }

    @Override
    public void execute() {
        if (manipulator.isStalling()) {
            manipulator.setState(Manipulator.State.IDLE);
        } else {
            manipulator.setState(Manipulator.State.COLLECT);
        }
    }

    @Override
    public void end(boolean interrupted) {
        manipulator.setState(Manipulator.State.IDLE);
    }
}