package org.team498.C2023.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team498.C2023.Robot;
import org.team498.C2023.RobotContainer;
import org.team498.C2023.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

import static org.team498.C2023.Constants.DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND;

public class OffenseDrive extends CommandBase {
    private final Drivetrain drivetrain = Drivetrain.getInstance();
    private final DoubleSupplier xTranslationSupplier;
    private final DoubleSupplier yTranslationSupplier;
    private final DoubleSupplier rotationSupplier;

    public OffenseDrive(DoubleSupplier xTranslationSupplier, DoubleSupplier yTranslationSupplier, DoubleSupplier rotationSupplier) {
        this.xTranslationSupplier = xTranslationSupplier;
        this.yTranslationSupplier = yTranslationSupplier;
        this.rotationSupplier = rotationSupplier;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        Robot.robotContainer.driver.setRightStickLastAngle(drivetrain.getYaw());
    }

    @Override
    public void execute() {
        double xTranslation = xTranslationSupplier.getAsDouble() * MAX_VELOCITY_METERS_PER_SECOND;
        double yTranslation = yTranslationSupplier.getAsDouble() * MAX_VELOCITY_METERS_PER_SECOND;
        double rotation = rotationSupplier.getAsDouble();

        // Set the target of the PID controller
        drivetrain.setSnapGoal(rotation);

        // Calculate the rotational speed from the pid controller, unless it's already at the goal
        double rotationalSpeed = drivetrain.calculateSnapSpeed();

        // Set the robot to drive in field relative mode, with the rotation controlled by the snap controller
        drivetrain.drive(xTranslation, yTranslation, rotationalSpeed * Robot.rotationDirection, true);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}