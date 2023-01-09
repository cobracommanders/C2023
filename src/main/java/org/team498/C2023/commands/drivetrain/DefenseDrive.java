package org.team498.C2023.commands.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team498.C2023.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

public class DefenseDrive extends CommandBase {
    private final Drivetrain drivetrain;

    private final DoubleSupplier xTranslationSupplier;
    private final DoubleSupplier yTranslationSupplier;
    private final DoubleSupplier rotationSupplier;

    public DefenseDrive(DoubleSupplier xTranslationSupplier, DoubleSupplier yTranslationSupplier, DoubleSupplier rotationSupplier) {

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

        // Set the robot to drive in field relative mode
        drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xTranslation, yTranslation, rotation, Rotation2d.fromDegrees(drivetrain.getYaw())));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds());
    }
}