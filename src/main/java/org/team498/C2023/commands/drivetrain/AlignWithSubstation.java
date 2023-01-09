package org.team498.C2023.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AlignWithSubstation extends CommandBase {
    public enum SubstationSide {
        LEFT, RIGHT
    }

    private final SubstationSide goal;

    public AlignWithSubstation(SubstationSide goal) {
        this.goal = goal;
    }

    @Override
    public void initialize() {
        if (goal == SubstationSide.LEFT)
            new SetPosition(0, 0, 0).schedule(); // TODO make this the actual correct cordinates for the left side
        else {
            new SetPosition(0, 0, 0).schedule(); // TODO make this the actual correct cordinates for the right side
        }

        // TODO add interrupt when joysticks are moved
    }
}
