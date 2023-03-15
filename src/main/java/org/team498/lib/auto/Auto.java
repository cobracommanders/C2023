package org.team498.lib.auto;

import org.team498.C2023.State;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public interface Auto {
    default boolean compare(Position start, Position end, Action action) {
        // return this.getStartPosition() == start && this.getEndPosition() == end && this.getAction() == action;
        return false;
    }
    public static enum Position {
        //TODO: Assign actual positions to these numbers
        CUBE_1,
        CUBE_2,
        CUBE_3,

        CONE_1,
        CONE_2,
        CONE_3,
        CONE_4,
        CONE_5,
        CONE_6,

        MARKER_1,
        MARKER_2,
        MARKER_3,
        MARKER_4,

        ENGAGE_GRID_1,
        ENGAGE_GRID_2,
        ENGAGE_FIELD_1,
        ENGAGE_FIELD_2,

        ENGAGED_1,
        ENGAGED_2,

        MOBILITY_1,
        MOBILITY_2
    }
    public static enum Action {
        INTAKE_CONE,
        SCORE_TOP_CONE,
        SCORE_MID_CONE,
        INTAKE_CUBE,
        SCORE_TOP_CUBE,
        SCORE_MID_CUBE,
        TRAVEL,
        ENGAGE
    }
    default String getName() {
        return getClass().getSimpleName();
    }
    // Position getStartPosition();
    // Position getEndPosition();
    // Action getAction();

    Command getCommand();
    Pose2d getInitialPose();
    State getInitialState();
}
