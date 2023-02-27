package org.team498.C2023.commands.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team498.C2023.commands.drivetrain.DriveToPosition;

import java.util.function.Supplier;

public class AlignAndExecute extends SequentialCommandGroup {
    public AlignAndExecute(Supplier<Pose2d> targetPosition) {
        super(new DriveToPosition(targetPosition),
              new Score());
    }
}
