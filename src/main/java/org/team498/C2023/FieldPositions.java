package org.team498.C2023;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import org.team498.lib.field.Point;
import org.team498.lib.field.Rectangle;
import org.team498.lib.field.Region;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldPositions {
    public static final double height = 8.0137;
    public static final double width = 16.54175;

    public static final double midline = width / 2;

    public static final Region redLoadingZone = new Region(
            new Rectangle(0, 6.75, 6.71, 1.265),
            new Rectangle(0, 6.75 - 1.24, 3.36, 1.24));

    public static final Region blueLoadingZone = new Region(
            flip(new Rectangle(0, 6.75, 6.71, 1.265)),
            flip(new Rectangle(0, 6.75 - 1.24, 3.36, 1.24)));

    public static final Rectangle blueChargeStation = new Rectangle(2.9, height - 6.4955, 1.95, 2.47);
    public static final Rectangle redChargeStation = flip(blueChargeStation);

    public static final Region blueCommunity = new Region(
            new Rectangle(1.37, height - 3.98, 1.985, 1.475),
            new Rectangle(1.37, 0, 3.54, 4.03));

    public static final Region redCommunity = new Region(
            flip(new Rectangle(1.37, height - 3.98, 1.985, 1.475)),
            flip(new Rectangle(1.37, 0, 3.54, 4.03)));

    public static final Point[] stagingMarkers = new Point[] {
            new Point(7.0775, height - 7.085),
            new Point(7.0775, height - 5.862),
            new Point(7.0775, height - 4.6396),
            new Point(7.0775, height - 3.4219),

            flip(new Point(7.0775, height - 7.085)),
            flip(new Point(7.0775, height - 5.862)),
            flip(new Point(7.0775, height - 4.6396)),
            flip(new Point(7.0775, height - 3.4219)),
    };

    public static class Grid {
        // topConeNodeL, topCubeNode, topConeNodeR,
        // midConeNodeL, midCubeNode, midConeNodeR,
        // hybridNodeL, hybridNodeM, hybridNodeR;

        private final double nodeSpacingHorizontal = 0.5576;
        private final double nodeSpacingVertical = 0.4328;

        private Point[][] allNodes = new Point[3][3];

        public Grid(Point topLeftConeNode, Alliance alliance) {
            for (int v = 0; v < allNodes.length; v++) {
                for (int h = 0; h < allNodes[0].length; h++) {
                    int i = alliance == Alliance.Blue ? h : 2 - h;

                    allNodes[v][i] = new Point(
                            topLeftConeNode.x + (nodeSpacingVertical * (alliance == Alliance.Blue ? v : -v)),
                            topLeftConeNode.y + (nodeSpacingHorizontal * h));
                }
            }
        }

        public Point[][] getNodePoints() {
            return Arrays.copyOf(allNodes, 3);
        }
    }

    public static Point flip(Point input) {
        return new Point(midline + (midline - input.x), input.y);
    }

    public static Rectangle flip(Rectangle input) {
        return new Rectangle(midline + (midline - input.height - input.x), input.y, input.height, input.width);
    }

    public static Grid blueLeft = new Grid(new Point(0.3657, height - 7.5033), Alliance.Blue);
    public static Grid blueCoOp = new Grid(new Point(0.3657, height - 5.8233), Alliance.Blue);
    public static Grid blueRight = new Grid(new Point(0.3657, height - 4.1503), Alliance.Blue);

    public static Grid redLeft = new Grid(flip(new Point(0.3657, height - 7.5033)), Alliance.Red);
    public static Grid redCoOp = new Grid(flip(new Point(0.3657, height - 5.8233)), Alliance.Red);
    public static Grid redRight = new Grid(flip(new Point(0.3657, height - 4.1503)), Alliance.Red);
    


    public static void displayAll() {
        redLoadingZone.displayOnDashboard("Red Loading Zone");
        blueLoadingZone.displayOnDashboard("Blue Loading Zone");

        blueChargeStation.displayOnDashboard("Blue Charge Station");
        redChargeStation.displayOnDashboard("Red Charge Station");

        blueCommunity.displayOnDashboard("Blue Community");
        redCommunity.displayOnDashboard("Red Community");

        List<Pose2d> stagingMarkerPoses = new LinkedList<>();
        for (Point marker : stagingMarkers) {
            stagingMarkerPoses.add(marker.toPose2d());
        }
        Robot.field.getObject("Staging Markers").setPoses(stagingMarkerPoses);

        List<Grid> allGrids = new LinkedList<>();
        List<Pose2d> blueOuterLeftNodePoses = new LinkedList<>();
        
        allGrids.add(blueLeft);
        allGrids.add(blueCoOp);
        allGrids.add(blueRight);        
        allGrids.add(redRight);        
        allGrids.add(redCoOp);
        allGrids.add(redLeft);

        for (Grid grid : allGrids) {
            for (int i = 0; i < 3; i++) {
                for (Point node : grid.getNodePoints()[i]) {
                    blueOuterLeftNodePoses.add(node.toPose2d());
                }
            }
        }
        Robot.field.getObject("Nodes").setPoses(blueOuterLeftNodePoses);
    }

    public static final Map<Integer, Pose3d> aprilTags = Map.of(
            1,
            new Pose3d(
                    Units.inchesToMeters(610.77),
                    Units.inchesToMeters(42.19),
                    Units.inchesToMeters(18.22),
                    new Rotation3d(0.0, 0.0, Math.PI)),
            2,
            new Pose3d(
                    Units.inchesToMeters(610.77),
                    Units.inchesToMeters(108.19),
                    Units.inchesToMeters(18.22),
                    new Rotation3d(0.0, 0.0, Math.PI)),
            3,
            new Pose3d(
                    Units.inchesToMeters(610.77),
                    Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
                    Units.inchesToMeters(18.22),
                    new Rotation3d(0.0, 0.0, Math.PI)),
            4,
            new Pose3d(
                    Units.inchesToMeters(636.96),
                    Units.inchesToMeters(265.74),
                    Units.inchesToMeters(27.38),
                    new Rotation3d(0.0, 0.0, Math.PI)),
            5,
            new Pose3d(
                    Units.inchesToMeters(14.25),
                    Units.inchesToMeters(265.74),
                    Units.inchesToMeters(27.38),
                    new Rotation3d()),
            6,
            new Pose3d(
                    Units.inchesToMeters(40.45),
                    Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
                    Units.inchesToMeters(18.22),
                    new Rotation3d()),
            7,
            new Pose3d(
                    Units.inchesToMeters(40.45),
                    Units.inchesToMeters(108.19),
                    Units.inchesToMeters(18.22),
                    new Rotation3d()),
            8,
            new Pose3d(
                    Units.inchesToMeters(40.45),
                    Units.inchesToMeters(42.19),
                    Units.inchesToMeters(18.22),
                    new Rotation3d()));

}