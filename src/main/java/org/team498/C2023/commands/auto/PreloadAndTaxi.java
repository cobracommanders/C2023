package org.team498.C2023.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team498.C2023.PathLib;
import org.team498.C2023.RobotState;
import org.team498.C2023.RobotState.GamePiece;
import org.team498.C2023.State;
import org.team498.C2023.commands.drivetrain.PathPlannerFollower;
import org.team498.C2023.commands.robot.Score;

public class PreloadAndTaxi extends SequentialCommandGroup {
    public PreloadAndTaxi() {
        super(
                new InstantCommand(() -> RobotState.getInstance().setCurrentGameMode(GamePiece.CUBE)),
                new InstantCommand(() -> RobotState.getInstance().setNextDriveteamState(State.TOP_CUBE)),
                new Score(),
                new PathPlannerFollower(PathLib.singleCubeTaxi));
    }
}
