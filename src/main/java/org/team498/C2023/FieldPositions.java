package org.team498.C2023;

import edu.wpi.first.math.geometry.Pose2d;
import org.team498.lib.field.RectangularRegion;

public class FieldPositions {
    public static final RectangularRegion LOADING_ZONE = new RectangularRegion(new Pose2d(), new Pose2d());
    public static final RectangularRegion COMMUNITY = new RectangularRegion(new Pose2d(), new Pose2d());

    public static final Pose2d SUBSTATION_LEFT_BLUE = new Pose2d();
    public static final Pose2d SUBSTATION_RIGHT_BLUE = new Pose2d();

}
