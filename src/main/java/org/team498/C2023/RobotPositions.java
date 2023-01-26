package org.team498.C2023;

import java.util.LinkedList;
import java.util.List;

import org.team498.C2023.Constants.DrivetrainConstants;
import org.team498.C2023.FieldPositions.Grid;
import org.team498.C2023.subsystems.Drivetrain;
import org.team498.lib.field.Point;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class RobotPositions {
    private static final Drivetrain drivetrain = Drivetrain.getInstance();
    private static final double scoringOffset = Units.inchesToMeters((DrivetrainConstants.ROBOT_WIDTH / 2) + 7);

    public enum Direction {
        LEFT, RIGHT
    }

    public static boolean inLoadingZone() {
        return drivetrain.isInRegion(
                Robot.alliance == Alliance.Blue
                        ? FieldPositions.blueLoadingZone
                        : FieldPositions.redLoadingZone);
    }

    public static boolean inCommunity() {
        return drivetrain.isInRegion(
                Robot.alliance == Alliance.Blue
                        ? FieldPositions.blueCommunity
                        : FieldPositions.redCommunity);
    }

    public static Pose2d getLeftPosition() {
        Pose2d pose = getClosestGrid().getNodePoints()[2][0].toPose2d();
        if (Robot.alliance == Alliance.Blue) {
            return pose.plus(new Transform2d(new Translation2d(scoringOffset, 0), Rotation2d.fromDegrees(180)));
        } else {
            return pose.plus(new Transform2d(new Translation2d(-scoringOffset, 0), Rotation2d.fromDegrees(0)));
        }
    }

    public static Pose2d getCenterPosition() {
        Pose2d pose = getClosestGrid().getNodePoints()[2][1].toPose2d();
        if (Robot.alliance == Alliance.Blue) {
            return pose.plus(new Transform2d(new Translation2d(scoringOffset, 0), Rotation2d.fromDegrees(180)));
        } else {
            return pose.plus(new Transform2d(new Translation2d(-scoringOffset, 0), Rotation2d.fromDegrees(0)));
        }
    }

    public static Pose2d getRightPosition() {
        Pose2d pose = getClosestGrid().getNodePoints()[2][2].toPose2d();
        if (Robot.alliance == Alliance.Blue) {
            return pose.plus(new Transform2d(new Translation2d(scoringOffset, 0), Rotation2d.fromDegrees(180)));
        } else {
            return pose.plus(new Transform2d(new Translation2d(-scoringOffset, 0), Rotation2d.fromDegrees(0)));
        }
    }

    private static Grid getClosestGrid() {
        LinkedList<FieldPositions.Grid> grids = Robot.alliance == Alliance.Blue
                ? FieldPositions.blueGrids
                : FieldPositions.redGrids;

        Grid closestGrid = null;

        Point currentPose = Point.fromPose2d(drivetrain.getPose());
        double currentX = currentPose.getX();
        double currentY = currentPose.getY();

        double closestDistance = Double.MAX_VALUE;

        for (Grid grid : grids) {
            Point centerPoint = grid.getNodePoints()[1][1];
            double x = centerPoint.getX();
            double y = centerPoint.getY();

            double xDiff = x - currentX;
            double yDiff = y - currentY;

            double distance = Math.hypot(xDiff, yDiff);

            if (distance < closestDistance) {
                closestDistance = distance;
                closestGrid = grid;
            }
        }

        return closestGrid;
    }

    private static Point getClosestScoringLocation() {
        LinkedList<FieldPositions.Grid> grids = Robot.alliance == Alliance.Blue
                ? FieldPositions.blueGrids
                : FieldPositions.redGrids;

        LinkedList<Point> scoringLocations = new LinkedList<>();
        grids.forEach(grid -> scoringLocations.addAll(List.of(grid.getNodePoints()[2])));

        Point closestPoint = null;

        Point currentPose = Point.fromPose2d(drivetrain.getPose());
        double currentX = currentPose.getX();
        double currentY = currentPose.getY();

        double closestDistance = Double.MAX_VALUE;

        for (Point point : scoringLocations) {
            double x = point.getX();
            double y = point.getY();

            double xDiff = x - currentX;
            double yDiff = y - currentY;

            double distance = Math.hypot(xDiff, yDiff);

            if (distance < closestDistance) {
                closestDistance = distance;
                closestPoint = point;
            }
        }

        return closestPoint;
    }
}
