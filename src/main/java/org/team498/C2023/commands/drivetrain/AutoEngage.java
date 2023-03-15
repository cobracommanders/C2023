package org.team498.C2023.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.RepeatCommand;

public class AutoEngage extends RepeatCommand {
    public AutoEngage() {
        super(
            new DriveToPosition(() -> new Pose2d(3.83, 2.9, Rotation2d.fromDegrees(180)))
        );
    }
}