package org.team498.C2023;

import edu.wpi.first.math.util.Units;
import org.team498.C2023.Constants.DrivetrainConstants;
import org.team498.C2023.subsystems.Drivetrain;

public class RobotPositions {
    private static final Drivetrain drivetrain = Drivetrain.getInstance();
    public static final double scoringOffset = Units.inchesToMeters((DrivetrainConstants.ROBOT_WIDTH / 2) + 10);
}