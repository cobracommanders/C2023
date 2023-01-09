package org.team498.C2023;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team498.C2023.commands.drivetrain.CameraDrive;
import org.team498.C2023.commands.drivetrain.DefenseDrive;
import org.team498.C2023.commands.drivetrain.FollowTrajectory;
import org.team498.C2023.commands.drivetrain.OffenseDrive;
import org.team498.C2023.subsystems.Drivetrain;
import org.team498.lib.drivers.Gyro;
import org.team498.lib.drivers.Xbox;
import org.team498.lib.util.Trajectories;

import java.util.function.DoubleSupplier;

import static org.team498.C2023.Constants.OIConstants;

public class RobotContainer {
    public static final Xbox xbox = new Xbox(OIConstants.DRIVER_CONTROLLER_ID);

    Drivetrain drivetrain = Drivetrain.getInstance();

    public RobotContainer() {
        Gyro gyro = Gyro.getInstance();

        drivetrain.setInitialPose(new Pose2d(8, 4, Rotation2d.fromDegrees(0)));
        xbox.setDeadzone(0.2);
        xbox.setTriggerThreshold(0.2);
        xbox.setRightStickLastAngle(gyro.getAngleOffset());
        xbox.setLeftStickLastAngle(gyro.getAngleOffset());

        configureCommands();

        Trigger robotInLoadingZone = new Trigger(() -> FieldPositions.LOADING_ZONE.containsPosition(drivetrain.getPose()));
    }

    private void configureCommands() {
        // Default drivetrain to offense mode
        drivetrain.setDefaultCommand(new OffenseDrive(xbox::leftX, xbox::leftY, xbox::rightAngle));

        // Toggle drivetrain to defense mode when X is pressed
        xbox.X().toggleOnTrue(new DefenseDrive(xbox::leftX, xbox::leftY, xbox::rightX));

        // Toggle drivetrain to camera mode when Y is pressed
        xbox.Y().toggleOnTrue(new CameraDrive(xbox::leftX, xbox::leftY, xbox::rightX));

        // Reset the gyro sensor when A is pressed
        xbox.A().onTrue(new InstantCommand(() -> Gyro.getInstance().reset()));


        xbox.start().onTrue(new FollowTrajectory(Trajectories.getTrajectory("test")));
    }

    public DoubleSupplier test() {
        return xbox::POVAngle;
    }
}