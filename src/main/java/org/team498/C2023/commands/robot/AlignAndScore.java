package org.team498.C2023.commands.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team498.C2023.commands.drivetrain.DriveToPosition;

import java.util.function.Supplier;

public class AlignAndScore extends SequentialCommandGroup {
    public AlignAndScore(Supplier<Pose2d> scoringPosition) {
        super(new DriveToPosition(scoringPosition),
              new Score()
              );
    }
}
