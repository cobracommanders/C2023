package org.team498.C2023.commands.drivetrain;

import org.team498.C2023.Robot;
import org.team498.C2023.subsystems.Drivetrain;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PathPlannerFollower extends CommandBase {
    private final Drivetrain drivetrain;
    private final PathPlannerTrajectory trajectory;
    private final Timer timer = new Timer();


    public PathPlannerFollower(PathPlannerTrajectory trajectory) {
        this.drivetrain = Drivetrain.getInstance();
        this.trajectory = trajectory;
        addRequirements(drivetrain);
    }
 
    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        drivetrain.setInitialPose(trajectory.getInitialHolonomicPose());
    }

    @Override
    public void execute() {
        PathPlannerState state = (PathPlannerState) trajectory.sample(timer.get());

        drivetrain.setPositionGoal(new Pose2d(state.poseMeters.getX(), state.poseMeters.getY(), state.holonomicRotation));

        drivetrain.driveToPositionGoals();
    }

    @Override
    public boolean isFinished() {
        return timer.get() > trajectory.getTotalTimeSeconds();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
