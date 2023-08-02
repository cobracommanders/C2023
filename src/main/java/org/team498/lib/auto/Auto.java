package org.team498.lib.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public interface Auto {
    default boolean compare(Position start, Position end, Action action) {
        // return this.getStartPosition() == start && this.getEndPosition() == end && this.getAction() == action;
        return false;
    }
    public static enum Position {
        
    }
    public static enum Action {
        
    }
    default String getName() {
        return getClass().getSimpleName();
    }
    // Position getStartPosition();
    // Position getEndPosition();
    // Action getAction();

    Command getCommand();
    Pose2d getInitialPose();
    //State getInitialState();
}
