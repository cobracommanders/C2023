package org.team498.C2023.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import org.team498.C2023.RobotContainer;
import org.team498.C2023.subsystems.Drivetrain;

public class DriveToPosition extends CommandBase {
    private final Drivetrain drivetrain;
    private final Pose2d pose;

    public DriveToPosition(Pose2d pose) {
        this.drivetrain = Drivetrain.getInstance();
        this.pose = pose;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.setPositionGoal(pose);
    }

    @Override
    public void execute() {
        drivetrain.driveToPositionGoals();
    }

    @Override
    public boolean isFinished() {
        return drivetrain.atPositionGoals() || RobotContainer.xbox.getLeftStickActive() || RobotContainer.xbox.getRightStickActive();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
