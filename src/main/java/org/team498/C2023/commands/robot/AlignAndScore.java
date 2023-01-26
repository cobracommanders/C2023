package org.team498.C2023.commands.robot;

import java.util.function.Supplier;

import org.team498.C2023.RobotState;
import org.team498.C2023.RobotState.GamePiece;
import org.team498.C2023.commands.drivetrain.DriveToPosition;
import org.team498.C2023.commands.outtake.OuttakeGamePiece;
import org.team498.C2023.subsystems.Elevator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AlignAndScore extends SequentialCommandGroup {
    public AlignAndScore(Supplier<Pose2d> scoringPosition) {
        GamePiece gamePiece = RobotState.getInstance().getCurrentGamePiece();

        addCommands(
            new ParallelCommandGroup(
                new DriveToPosition(scoringPosition)
                // , Elevator.getInstance().setPosition(Elevator.getInstance().nextHeight)
            )
            // , new OuttakeGamePiece(gamePiece)
        );
    }
}
