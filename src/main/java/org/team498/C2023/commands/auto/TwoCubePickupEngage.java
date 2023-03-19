package org.team498.C2023.commands.auto;

import org.team498.C2023.PathLib;
import org.team498.C2023.RobotState;
import org.team498.C2023.State;
import org.team498.C2023.RobotState.GameMode;
import org.team498.C2023.RobotState.ScoringOption;
import org.team498.C2023.commands.SetRobotState;
import org.team498.C2023.commands.drivetrain.AutoEngageBangBang;
import org.team498.C2023.commands.drivetrain.BangBangEngage;
import org.team498.C2023.commands.drivetrain.LockWheels;
import org.team498.C2023.commands.drivetrain.PathPlannerFollower;
import org.team498.C2023.commands.elevator.SetElevatorToNextState;
import org.team498.C2023.commands.elevatorwrist.SetElevatorWristToNextState;
import org.team498.C2023.commands.intakerollers.SetIntakeRollersToNextState;
import org.team498.C2023.commands.intakewrist.SetIntakeWristToNextState;
import org.team498.C2023.commands.manipulator.SetManipulatorToNextState;
import org.team498.C2023.commands.robot.FullScore;
import org.team498.C2023.commands.robot.ReturnToIdle;
import org.team498.lib.auto.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TwoCubePickupEngage implements Auto {
    @Override
    public Command getCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> RobotState.getInstance().setCurrentGameMode(GameMode.CUBE)),
            new InstantCommand(() -> RobotState.getInstance().setNextScoringOption(ScoringOption.TOP)),
            new FullScore(),
            new SetRobotState(State.GROUND_CUBE),
            new ParallelCommandGroup(
                new PathPlannerFollower(PathLib.secondNodeToTopCube),
                new SetElevatorToNextState(),
                new SetElevatorWristToNextState(),
                new SequentialCommandGroup(
                    new WaitCommand(1),
                    new ParallelCommandGroup(
                        new SetIntakeRollersToNextState(),
                        new SetIntakeWristToNextState(),
                        new SetManipulatorToNextState()
                    )
                )
            ),
            new ParallelCommandGroup(
                new PathPlannerFollower(PathLib.topCubeToSecondNode),
                new SequentialCommandGroup(
                    new WaitCommand(0.5),
                    new ReturnToIdle(),
                    new InstantCommand(() -> RobotState.getInstance().setCurrentGameMode(GameMode.CUBE)),
                    new InstantCommand(() -> RobotState.getInstance().setNextScoringOption(ScoringOption.MID))
                )
            ),
            new LockWheels(),
            new FullScore(),
            new PathPlannerFollower(PathLib.secondNodeToSecondCube),
            new AutoEngageBangBang()
        );
    }

    @Override
    public Pose2d getInitialPose() {
        return PathLib.secondNodeToTopCube.getInitialHolonomicPose();
    }
    
    @Override
    public State getInitialState() {
        return State.IDLE_CUBE;
    }
}
