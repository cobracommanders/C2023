package org.team498.C2023.commands.sequences;

import org.team498.C2023.commands.drivetrain.DriveToPosition;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AlignWithSubstation extends SequentialCommandGroup {
    public AlignWithSubstation() {
        super(new DriveToPosition(new Pose2d(0,0, new Rotation2d())));
    }
}
