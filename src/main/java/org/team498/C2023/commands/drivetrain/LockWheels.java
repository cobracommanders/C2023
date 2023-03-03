package org.team498.C2023.commands.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import org.team498.C2023.subsystems.Drivetrain;

public class LockWheels extends InstantCommand {
    private final Drivetrain drivetrain = Drivetrain.getInstance();

    private final SwerveModuleState[] lockedStates = new SwerveModuleState[] {
            // Front left
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            // Front right
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            // Back left
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            // Back right
            new SwerveModuleState(0, Rotation2d.fromDegrees(45))};

    public LockWheels() {
        addRequirements(this.drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.setModuleStates(lockedStates, true);
    }
}
