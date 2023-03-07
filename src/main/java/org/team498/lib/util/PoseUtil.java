package org.team498.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public class PoseUtil {
    public static Pose2d toPose2d(Pose3d pose) {
        return new Pose2d(pose.getX(), pose.getY(), pose.getRotation().toRotation2d());
    }
}
