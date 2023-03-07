package org.team498.C2023.commands.auto;

import org.team498.C2023.RobotState;
import org.team498.C2023.RobotState.GamePiece;
import org.team498.C2023.State;
import org.team498.C2023.commands.robot.Score;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class JustScore extends SequentialCommandGroup {
    public JustScore() {
        super(
                new InstantCommand(() -> RobotState.getInstance().setCurrentGameMode(GamePiece.CUBE)),
                new InstantCommand(() -> RobotState.getInstance().setNextDriveteamState(State.TOP_CUBE)),
                new Score());
    }
}
