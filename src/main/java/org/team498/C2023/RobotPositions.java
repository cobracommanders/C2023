package org.team498.C2023;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.team498.C2023.Constants.DrivetrainConstants;
import org.team498.C2023.FieldPositions.Grid;
import org.team498.C2023.subsystems.Drivetrain;
import org.team498.lib.field.Point;

import java.util.LinkedList;

public class RobotPositions {
    private static final Drivetrain drivetrain = Drivetrain.getInstance();
    public static final double scoringOffset = Units.inchesToMeters((DrivetrainConstants.ROBOT_WIDTH / 2) + 10);

    public static boolean inCommunity() {
        return drivetrain.isInRegion(Robot.alliance == Alliance.Blue
                                     ? FieldPositions.blueCommunity
                                     : FieldPositions.redCommunity);
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