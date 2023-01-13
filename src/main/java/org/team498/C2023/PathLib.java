package org.team498.C2023;

import edu.wpi.first.math.trajectory.Trajectory;
import org.team498.lib.util.Trajectories;

import com.pathplanner.lib.PathPlannerTrajectory;

public class PathLib {
    public static final Trajectory unnamed = Trajectories.getTrajectory("Unnamed");
    public static final PathPlannerTrajectory pathPlannerTrajectory = Trajectories.getPathPlannerTrajectory("New Path");
}
