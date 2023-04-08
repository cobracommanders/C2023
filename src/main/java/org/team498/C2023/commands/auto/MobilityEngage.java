package org.team498.C2023.commands.auto;

import org.team498.C2023.PathLib;
import org.team498.C2023.RobotState;
import org.team498.C2023.State;
import org.team498.C2023.RobotState.GameMode;
import org.team498.C2023.RobotState.ScoringOption;
import org.team498.C2023.commands.drivetrain.PathPlannerFollower;
import org.team498.C2023.commands.drivetrain.chargestation.BangBangEngage;
import org.team498.C2023.commands.robot.FullScore;
import org.team498.lib.auto.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class MobilityEngage implements Auto {

    @Override
    public Command getCommand() {
        return new SequentialCommandGroup(
            // new InstantCommand(() -> RobotState.getInstance().setCurrentGameMode(GameMode.CUBE)),
            // new InstantCommand(() -> RobotState.getInstance().setNextScoringOption(ScoringOption.TOP)),
            // new FullScore(),
            new PathPlannerFollower(PathLib.mobilityEngage)//,
            // new BangBangEngage(false)
        );
    }

    @Override
    public Pose2d getInitialPose() {
        return PathLib.mobilityEngage.getInitialHolonomicPose();
    }

    @Override
    public State getInitialState() {
        return State.IDLE_CUBE;
    }
    
}
