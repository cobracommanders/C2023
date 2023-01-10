package org.team498.C2023.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team498.C2023.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

public class OffenseDrive extends CommandBase {
    private final Drivetrain drivetrain;
    private final DoubleSupplier xTranslationSupplier;
    private final DoubleSupplier yTranslationSupplier;
    private final DoubleSupplier rotationSupplier;

    public OffenseDrive(DoubleSupplier xTranslationSupplier, DoubleSupplier yTranslationSupplier, DoubleSupplier rotationSupplier) {

        this.drivetrain = Drivetrain.getInstance();
        this.xTranslationSupplier = xTranslationSupplier;
        this.yTranslationSupplier = yTranslationSupplier;
        this.rotationSupplier = rotationSupplier;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double xTranslation = xTranslationSupplier.getAsDouble();
        double yTranslation = yTranslationSupplier.getAsDouble();
        double rotation = rotationSupplier.getAsDouble();

        // Set the target of the PID controller
        drivetrain.setSnapGoal(rotation);
        // Calculate the rotational speed from the pid controller, unless it's already at the goal
        double rotationalSpeed = drivetrain.calculateSnapSpeed();

        // Set the robot to drive in field relative mode, with the rotation controlled by the snap controller
        drivetrain.drive(xTranslation, yTranslation, rotationalSpeed, true);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}