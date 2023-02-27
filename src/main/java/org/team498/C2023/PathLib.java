package org.team498.C2023;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.trajectory.Trajectory;
import org.team498.lib.util.Trajectories;

public class PathLib {
    public static final Trajectory unnamed = Trajectories.getTrajectory("Unnamed");
    public static final PathPlannerTrajectory pathPlannerTrajectory = Trajectories.getPathPlannerTrajectory("New Path");

    public static final class SingleCube {
        public static final PathPlannerTrajectory Path1 = Trajectories.getPathPlannerTrajectory("camera");
    }
}
