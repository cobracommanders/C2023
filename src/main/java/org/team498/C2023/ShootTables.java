package org.team498.C2023;

import org.team498.lib.util.LinearInterpolator;

public class ShootTables {
    // Number of robot loop cycles to offset the robot position by when calculating
    // the angle to aim at
    public static final double aimLagCompensation = 10;

    public static class midCube {
        public static final LinearInterpolator shooterRPM = new LinearInterpolator(new double[][] {
                { 0.5, 0.5 },
                { 0.6, 0.6 },
                { 0.7, 0.7 },
                { 0.8, 0.8 },
                { 0.9, 0.9 }
        });
        public static final LinearInterpolator wristAngle = new LinearInterpolator(new double[][] {
                { 0.5, -0.1 },
                { 0.6, -0.05 },
                { 0.7, 0 },
                { 0.8, 0.05 },
                { 0.9, 0.1 }
        });
        public static final LinearInterpolator elevatorHeight = new LinearInterpolator(new double[][] {
                { 0.5, 0.5 },
                { 0.6, 0.6 },
                { 0.7, 0.7 },
                { 0.8, 0.8 },
                { 0.9, 0.875 }
        });
    }

    public static class topCube {
        public static final LinearInterpolator shooterRPM = new LinearInterpolator(new double[][] {
                { 0.5, 0.5 },
                { 0.6, 0.6 },
                { 0.7, 0.7 },
                { 0.8, 0.8 },
                { 0.9, 0.9 }
        });
        public static final LinearInterpolator wristAngle = new LinearInterpolator(new double[][] {
                { 0.5, -0.1 },
                { 0.6, -0.05 },
                { 0.7, 0 },
                { 0.8, 0.05 },
                { 0.9, 0.1 }
        });
        public static final LinearInterpolator elevatorHeight = new LinearInterpolator(new double[][] {
                { 0.5, 0.5 },
                { 0.6, 0.6 },
                { 0.7, 0.7 },
                { 0.8, 0.8 },
                { 0.9, 0.875 }
        });
    }

    public static class midCone {
        public static final LinearInterpolator shooterRPM = new LinearInterpolator(new double[][] {
                { 0.5, 0.5 },
                { 0.6, 0.6 },
                { 0.7, 0.7 },
                { 0.8, 0.8 },
                { 0.9, 0.9 }
        });
        public static final LinearInterpolator wristAngle = new LinearInterpolator(new double[][] {
                { 0.5, -0.1 },
                { 0.6, -0.05 },
                { 0.7, 0 },
                { 0.8, 0.05 },
                { 0.9, 0.1 }
        });
        public static final LinearInterpolator elevatorHeight = new LinearInterpolator(new double[][] {
                { 0.5, 0.5 },
                { 0.6, 0.6 },
                { 0.7, 0.7 },
                { 0.8, 0.8 },
                { 0.9, 0.875 }
        });
    }
}
