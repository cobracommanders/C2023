
package org.team498.C2023.commands.auto;

import org.team498.C2023.commands.drivetrain.TimedDrive;
import org.team498.lib.wpilib.ChassisSpeeds;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreAndTaxi extends SequentialCommandGroup {
    public ScoreAndTaxi() {
        super(
            new JustScore()//,
            // new TimedDrive(new ChassisSpeeds(1, 0, 0), 3)
        );
    }
}