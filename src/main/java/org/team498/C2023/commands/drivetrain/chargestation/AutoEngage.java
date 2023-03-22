package org.team498.C2023.commands.drivetrain.chargestation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;

import org.team498.C2023.Robot;
import org.team498.C2023.subsystems.Drivetrain;
import org.team498.lib.util.PoseUtil;

public class AutoEngage extends CommandBase {
    private final Drivetrain drivetrain = Drivetrain.getInstance();
    private final double chargingStationPoseX = 3.93;

    public AutoEngage() {
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.setPositionGoal(Robot.alliance == Alliance.Blue ? new Pose2d(chargingStationPoseX, 0, Rotation2d.fromDegrees(180)) : PoseUtil.flip(new Pose2d(chargingStationPoseX, 0, Rotation2d.fromDegrees(180))));
    }

    @Override
    public void execute() {
        drivetrain.drive(drivetrain.calculatePositionSpeed().vxMetersPerSecond, 0, drivetrain.calculateAngleSpeed(), true);
    }

    @Override
    public boolean isFinished() {
        return drivetrain.atXGoal();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
