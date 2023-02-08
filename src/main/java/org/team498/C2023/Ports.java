package org.team498.C2023;

public final class Ports {

    public static class Drivetrain {
        public static final int FL_DRIVE_ID = 3;
        public static final int FR_DRIVE_ID = 5;
        public static final int BL_DRIVE_ID = 1;
        public static final int BR_DRIVE_ID = 7;

        public static final int FL_STEER_ID = 4;
        public static final int FR_STEER_ID = 6;
        public static final int BL_STEER_ID = 2;
        public static final int BR_STEER_ID = 8;

        public static final int FL_CANCODER_ID = 12;
        public static final int FR_CANCODER_ID = 13;
        public static final int BL_CANCODER_ID = 11;
        public static final int BR_CANCODER_ID = 14;

        public static final int GYRO = 20;
    }

    public static class Elevator {
        public static final int L_ELEVATOR_ID = 31;
        public static final int R_ELEVATOR_ID = 32;
        public static final int ELEVATOR_LIMIT = 0;
    }

    public static class Manipulator {
        public static final int ROLLERS = 33;
        public static final int OUTTAKE_BOTTOM = 34;
        public static final int WRIST = 35;
        public static final int BEAM_BREAK = 3;
    }

    public static class Intake {
        public static final int ROLLERS = 40;
        public static final int L_WRIST = 41;
        public static final int R_WRIST = 42;
    }

}
