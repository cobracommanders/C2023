package org.team498.lib.field;

import edu.wpi.first.math.geometry.Pose2d;

public class RectangularRegion {

    Pose2d bottomLeft;
    Pose2d upperRight;

    /**
     * @param bottomLeft The bottom left corner of the area
     * @param upperRight The upper right corner of the area
     */
    public RectangularRegion(Pose2d bottomLeft, Pose2d upperRight) {
        if (bottomLeft.getY() > upperRight.getY() || bottomLeft.getX() > upperRight.getX()) {

            new Exception("Insert valid coordinates").printStackTrace();
            return;
        }

        this.bottomLeft = bottomLeft;
        this.upperRight = upperRight;
    }

    public Pose2d getBottomLeft() {
        return bottomLeft;
    }
    public Pose2d getUpperRight() {
        return upperRight;
    }

    public boolean containsPosition(Pose2d pose) {
        double x = pose.getX();
        double y = pose.getY();

        return (x >= bottomLeft.getX() && x <= upperRight.getX() && y >= bottomLeft.getY() && y <= upperRight.getY());
    }

}
