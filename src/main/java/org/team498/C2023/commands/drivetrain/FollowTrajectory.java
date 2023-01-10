package org.team498.C2023.commands.drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team498.C2023.subsystems.Drivetrain;

public class FollowTrajectory extends CommandBase {
    private final Drivetrain drivetrain;
    private final Trajectory trajectory;
    private final Timer timer = new Timer();

    public FollowTrajectory(Trajectory trajectory) {
        this.drivetrain = Drivetrain.getInstance();
        this.trajectory = trajectory;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        Trajectory.State goal = trajectory.sample(timer.get());

        ChassisSpeeds speeds = drivetrain.getSpeedsFromTrajectoryState(goal);
        drivetrain.drive(speeds, new Translation2d());
    }

    @Override
    public boolean isFinished() {
        return drivetrain.atTrajectoryGoal();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
