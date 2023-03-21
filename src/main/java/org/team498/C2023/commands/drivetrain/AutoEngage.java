package org.team498.C2023.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;

import org.team498.C2023.Robot;
import org.team498.C2023.subsystems.Drivetrain;
import org.team498.lib.util.PoseUtil;

public class AutoEngage extends CommandBase {
    private final Drivetrain drivetrain = Drivetrain.getInstance();

    public AutoEngage() {
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.setPositionGoal(Robot.alliance == Alliance.Blue ? new Pose2d(3.93, 2.9, Rotation2d.fromDegrees(180)) : PoseUtil.flip(new Pose2d(3.93, 2.9, Rotation2d.fromDegrees(180))));
    }

    @Override
    public void execute() {
        drivetrain.drive(drivetrain.calculateTranslationalSpeeds().vxMetersPerSecond, 0, drivetrain.calculateRotationalSpeed(), true);
    }

    @Override
    public boolean isFinished() {
        return drivetrain.atXControllerGoal();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
