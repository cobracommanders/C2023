package org.team498.C2023.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team498.C2023.Robot;
import org.team498.C2023.subsystems.Drivetrain;
import org.team498.lib.util.RotationUtil;

import java.util.function.DoubleSupplier;

import static org.team498.C2023.Constants.DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND;

public class TargetDrive extends CommandBase {
    private final Drivetrain drivetrain;
    private final DoubleSupplier xTranslationSupplier;
    private final DoubleSupplier yTranslationSupplier;
    private final Pose2d target;

    public TargetDrive(DoubleSupplier xTranslationSupplier, DoubleSupplier yTranslationSupplier, Pose2d target) {
        this.drivetrain = Drivetrain.getInstance();
        this.xTranslationSupplier = xTranslationSupplier;
        this.yTranslationSupplier = yTranslationSupplier;
        this.target = target;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double xTranslation = xTranslationSupplier.getAsDouble() * MAX_VELOCITY_METERS_PER_SECOND;
        double yTranslation = yTranslationSupplier.getAsDouble() * MAX_VELOCITY_METERS_PER_SECOND;

        // Set the target of the PID controller
        drivetrain.setSnapGoal(calculateDegreesToTarget(target));

        // Calculate the rotational speed from the pid controller, unless it's already at the goal
        double rotationalSpeed = drivetrain.calculateSnapSpeed();

        // Set the robot to drive in field relative mode, with the rotation controlled by the snap controller
        drivetrain.drive(xTranslation, yTranslation, rotationalSpeed, true);
    }

    public double calculateDegreesToTarget(Pose2d target) {
        Pose2d currentPose = drivetrain.getPose();

        // Estimate the future pose of the robot in 10 loop cycles to compensate for lag
        double newX = currentPose.getX() + (drivetrain.currentSpeeds.vxMetersPerSecond * (Robot.kDefaultPeriod * 10));
        double newY = currentPose.getY() + (drivetrain.currentSpeeds.vyMetersPerSecond * (Robot.kDefaultPeriod * 10));

        Pose2d futurePose = new Pose2d(newX, newY, new Rotation2d());

        // Calculate the angle between the target and the current robot position.
        double angle = Math.toDegrees(Math.atan2(-futurePose.getY() + target.getY(), -futurePose.getX() + target.getX()));

        // Normalize the angle to a number between 0 and 360.
        angle = RotationUtil.toSignedDegrees(angle);

        // Return the angle to which the turret needs to be adjusted.
        return angle;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}