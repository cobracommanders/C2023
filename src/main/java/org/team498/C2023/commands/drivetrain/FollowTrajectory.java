package org.team498.C2023.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team498.C2023.Robot;
import org.team498.C2023.subsystems.Drivetrain;

public class FollowTrajectory extends CommandBase {
    private final Drivetrain drivetrain = Drivetrain.getInstance();
    private final Trajectory trajectory;
    private final Timer timer = new Timer();

    public FollowTrajectory(Trajectory trajectory) {
        this.trajectory = trajectory;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        drivetrain.setPose(trajectory.getInitialPose());
    }

    @Override
    public void execute() {
        Trajectory.State state = trajectory.sample(timer.get());

        drivetrain.setPositionGoal(new Pose2d(state.poseMeters.getX(), state.poseMeters.getY(), new Rotation2d()));

        drivetrain.driveToPositionGoals();
    }

    @Override
    public boolean isFinished() {
        return timer.get() > trajectory.getTotalTimeSeconds() || Robot.controls.driver.isStickActive();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
