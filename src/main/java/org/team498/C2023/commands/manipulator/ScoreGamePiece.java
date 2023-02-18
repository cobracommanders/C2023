package org.team498.C2023.commands.manipulator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team498.C2023.RobotState;
import org.team498.C2023.subsystems.Manipulator;

public class ScoreGamePiece extends CommandBase {
    private final Manipulator manipulator = Manipulator.getInstance();

    @Override
    public void initialize() {
        if (RobotState.getInstance().hasCone()) {
            manipulator.setState(Manipulator.State.SCORE_CONE);
        } else {
            manipulator.setState(Manipulator.State.SCORE_CUBE);
        }
    }
}
