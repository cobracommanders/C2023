package org.team498.C2023.commands.manipulator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team498.C2023.RobotState;
import org.team498.C2023.subsystems.Manipulator;

public class HoldCone extends CommandBase {
    private final Manipulator manipulator = Manipulator.getInstance();
    private final RobotState robotState = RobotState.getInstance();

    public HoldCone() {
        addRequirements(manipulator);
    }

    @Override
    public void execute() {
        if (manipulator.isStalling() || robotState.hasCube()) {
            manipulator.setState(Manipulator.State.IDLE);
        } else {
            manipulator.setState(Manipulator.State.COLLECT_CONE);
        }
    }

    @Override
    public void end(boolean interrupted) {
        manipulator.setState(Manipulator.State.IDLE);
    }
}