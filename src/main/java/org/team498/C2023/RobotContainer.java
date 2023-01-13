package org.team498.C2023;

import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;

import org.team498.C2023.Constants.OIConstants;
import org.team498.C2023.commands.drivetrain.CameraDrive;
import org.team498.C2023.commands.drivetrain.DefenseDrive;
import org.team498.C2023.commands.drivetrain.OffenseDrive;
import org.team498.C2023.commands.drivetrain.PathPlannerFollower;
import org.team498.C2023.commands.drivetrain.TargetDrive;
import org.team498.C2023.subsystems.Drivetrain;
import org.team498.lib.drivers.Gyro;
import org.team498.lib.drivers.Xbox;
import org.team498.lib.field.Point;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    public static final Xbox xbox = new Xbox(OIConstants.DRIVER_CONTROLLER_ID);

    Drivetrain drivetrain = Drivetrain.getInstance();

    public RobotContainer() {
        Gyro gyro = Gyro.getInstance();

        drivetrain.setInitialPose(new Pose2d(8.29, 4, Rotation2d.fromDegrees(0)));
        xbox.setDeadzone(0.2);
        xbox.setTriggerThreshold(0.2);
        xbox.setRightStickLastAngle(-gyro.getAngleOffset());

        configureCommands();

    }

    private void configureCommands() {
        // Default drivetrain to offense mode
        //drivetrain.setDefaultCommand(new OffenseDrive(xbox::leftX, xbox::leftY, xbox::rightAngle));

        Trigger robotInLoadingZone = new Trigger(() -> FieldPositions.blueLoadingZone.contains(Point.fromPose2d(drivetrain.getPose())));

        //xbox.B().and(robotInLoadingZone).onTrue(new DriveToPosition(FieldPositions.SUBSTATION_RIGHT_BLUE));


        drivetrain.setDefaultCommand(new OffenseDrive(xbox::leftY, xbox::leftX, xbox::rightAngle));

        xbox.A().toggleOnTrue(new TargetDrive(xbox::leftY, xbox::leftX, new Pose2d(6, 4, new Rotation2d())));

        xbox.start().onTrue(new PathPlannerFollower(PathLib.pathPlannerTrajectory));        

        List<Pose2d> poses = new LinkedList<>();
        List<Trajectory.State> trajectoryStates = PathLib.pathPlannerTrajectory.getStates();
        for (int i = 0; i < trajectoryStates.size(); i += trajectoryStates.size() / 84) {
            poses.add(trajectoryStates.get(i).poseMeters);
        }
        Robot.field.getObject("Trajectory").setPoses(poses);


        // Toggle drivetrain to defense mode when X is pressed
        xbox.X().toggleOnTrue(new DefenseDrive(xbox::leftY, xbox::leftX, xbox::rightX));

        // Toggle drivetrain to camera mode when Y is pressed
        xbox.Y().toggleOnTrue(new CameraDrive(xbox::leftY, xbox::leftX, xbox::rightX));
        

        // Reset the gyro sensor when A is pressed
        //xbox.A().onTrue(new InstantCommand(() -> Gyro.getInstance().reset()));
    }

    public DoubleSupplier test() {
        return xbox::rightAngle;
    }
}