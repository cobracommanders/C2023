package org.team498.C2023;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.team498.C2023.Constants.DrivetrainConstants;
import org.team498.C2023.FieldPositions.Grid;
import org.team498.C2023.subsystems.Drivetrain;
import org.team498.lib.field.Point;
import org.team498.lib.field.Rectangle;

import java.util.LinkedList;

public class RobotPositions {
    private static final Drivetrain drivetrain = Drivetrain.getInstance();
    public static final double scoringOffset = Units.inchesToMeters((DrivetrainConstants.ROBOT_WIDTH / 2) + 10);

    private static final Point rightDoubleSubstationBlue = new Point(0, 0);
    private static final Point leftDoubleSubstationBlue = new Point(0, 0);
    private static final Point rightDoubleSubstationRed = FieldPositions.flip(rightDoubleSubstationBlue);
    private static final Point leftDoubleSubstationRed = FieldPositions.flip(leftDoubleSubstationBlue);

    private static final Point singleSubstationBlue = new Point(0, 0);
    private static final Point singleSubstationRed = FieldPositions.flip(singleSubstationBlue);

    private static final Rectangle blueSSPickupArea = new Rectangle(0, 0, 0, 0);
    private static final Rectangle redSSPickupArea = FieldPositions.flip(blueSSPickupArea);

    public static boolean inLoadingZone() {
        return drivetrain.isInRegion(Robot.alliance == Alliance.Blue
                                     ? FieldPositions.blueLoadingZone
                                     : FieldPositions.redLoadingZone);
    }

    public static boolean inCommunity() {
        return drivetrain.isInRegion(Robot.alliance == Alliance.Blue
                                     ? FieldPositions.blueCommunity
                                     : FieldPositions.redCommunity);
    }

    public static boolean inSSPickupArea() {
        return drivetrain.isInRegion(Robot.alliance == Alliance.Blue
                                     ? blueSSPickupArea
                                     : redSSPickupArea);
    }

    public static Pose2d getNextScoringNodePosition() {
        int height = switch (RobotState.getInstance().getNextScoringOption()) {
            case TOP -> 0;
            case MID -> 1;
            case SPIT -> 2;
        };

        Grid closestGrid = getClosestGrid();
        if (RobotState.getInstance().inCubeMode()) return closestGrid.getNodePoints()[height][1].toPose2d(); // Return the center position for cubes
        if (drivetrain.distanceTo(closestGrid.getNodePoints()[2][0]) < drivetrain.distanceTo(closestGrid.getNodePoints()[2][2]))
            return closestGrid.getNodePoints()[height][0].toPose2d();
        return closestGrid.getNodePoints()[height][2].toPose2d();
    }
    public static double getFutureScoringNodeDistance() {
        return Math.abs(drivetrain.distanceTo(Point.fromPose2d(getNextScoringNodePosition().transformBy(drivetrain.getVelocity().inverse()))));
    }

    public static Pose2d getClosestScoringPosition() {
        Grid closestGrid = getClosestGrid();
        if (RobotState.getInstance().inCubeMode()) return closestGrid.getNodePoints()[2][1].toPose2d(); // Return the center position for cubes
        if (drivetrain.distanceTo(closestGrid.getNodePoints()[2][0]) < drivetrain.distanceTo(closestGrid.getNodePoints()[2][2]))
            return closestGrid.getNodePoints()[2][0].toPose2d();
        return closestGrid.getNodePoints()[2][2].toPose2d();
    }

    public static Pose2d getLeftScoringPosition() {
        Pose2d pose = getClosestGrid().getNodePoints()[2][0].toPose2d();
        return Robot.alliance == Alliance.Blue
               ? pose.plus(new Transform2d(new Translation2d(scoringOffset, 0), Rotation2d.fromDegrees(180)))
               : pose.plus(new Transform2d(new Translation2d(-scoringOffset, 0), Rotation2d.fromDegrees(0)));
    }

    public static Pose2d getCenterScoringPosition() {
        Pose2d pose = getClosestGrid().getNodePoints()[2][1].toPose2d();
        return Robot.alliance == Alliance.Blue
               ? pose.plus(new Transform2d(new Translation2d(scoringOffset, 0), Rotation2d.fromDegrees(180)))
               : pose.plus(new Transform2d(new Translation2d(-scoringOffset, 0), Rotation2d.fromDegrees(0)));
    }

    public static Pose2d getRightScoringPosition() {
        Pose2d pose = getClosestGrid().getNodePoints()[2][2].toPose2d();
        return Robot.alliance == Alliance.Blue
               ? pose.plus(new Transform2d(new Translation2d(scoringOffset, 0), Rotation2d.fromDegrees(180)))
               : pose.plus(new Transform2d(new Translation2d(-scoringOffset, 0), Rotation2d.fromDegrees(0)));
    }

    public static Pose2d getRightSubstationPosition() {
        return Robot.alliance == Alliance.Blue
               ? rightDoubleSubstationBlue.toPose2d()
               : rightDoubleSubstationRed.toPose2d();
    }

    public static Pose2d getLeftSubstationPosition() {
        return Robot.alliance == Alliance.Blue
               ? leftDoubleSubstationBlue.toPose2d()
               : leftDoubleSubstationRed.toPose2d();
    }

    public static Pose2d getSingleSubstationPosition() {
        return Robot.alliance == Alliance.Blue
               ? singleSubstationBlue.toPose2d()
               : singleSubstationRed.toPose2d();
    }

    private static Grid getClosestGrid() {
        LinkedList<FieldPositions.Grid> grids = Robot.alliance == Alliance.Blue
                                                ? FieldPositions.blueGrids
                                                : FieldPositions.redGrids;

        Grid closestGrid = null;

        double closestDistance = Double.MAX_VALUE;

        for (Grid grid : grids) {
            double distance = drivetrain.distanceTo(grid.getNodePoints()[1][1]);

            if (distance < closestDistance) {
                closestDistance = distance;
                closestGrid = grid;
            }
        }

        return closestGrid;
    }
}