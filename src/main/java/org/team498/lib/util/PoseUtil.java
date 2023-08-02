package org.team498.lib.util;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class PoseUtil {
    public static Pose2d toPose2d(Pose3d pose) {
        return new Pose2d(pose.getX(), pose.getY(), pose.getRotation().toRotation2d());
    }

    public static Pose2d toPose2d(Transform2d pose) {
        return new Pose2d(pose.getX(), pose.getY(), pose.getRotation());
    }

    public static Pose2d toPose2d(Translation2d pose) {
        return new Pose2d(pose.getX(), pose.getY(), pose.getAngle());
    }
    //TODO: add General coordinates to Lib
    public static Pose2d flip(Pose2d input) {
        return new Pose2d();
        //return new Pose2d(FieldPositions.midline + (FieldPositions.midline - input.getX()), input.getY(), Rotation2d.fromDegrees((input.getRotation().getDegrees() + 180) * -1));
    }
}
