package org.team498.lib.util;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.IOException;
import java.nio.file.Path;

public class Trajectories {
    public static Trajectory getTrajectory(String name) {
        String filepath = "pathplanner/generatedJSON/" + name + ".wpilib.json";

        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filepath);
        try {
            return TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException e) {
            e.printStackTrace();
            return new Trajectory();
        }
    }

    public static PathPlannerTrajectory getPathPlannerTrajectory(String name) {
        return PathPlanner.loadPath(name, 5, 2);
    }
}
