package org.team498.C2023.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import org.team498.C2023.FieldPositions;
import org.team498.C2023.RobotPositions;
import org.team498.C2023.RobotState;
import org.team498.C2023.State;
import org.team498.C2023.RobotState.GameMode;
import org.team498.C2023.commands.SetRobotState;
import org.team498.C2023.commands.manipulator.SetManipulatorToNextState;
import org.team498.C2023.commands.robot.ReturnToIdle;
import org.team498.C2023.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import org.team498.lib.auto.Auto;

public class JustAutoShot implements Auto {
    @Override
    public Command getCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> RobotState.getInstance().setCurrentGameMode(GameMode.CUBE)),
                new InstantCommand(() -> Elevator.getInstance().updateInitialPosition(true)),
                new SetRobotState(State.AUTO_SHOT),
                new SetManipulatorToNextState(),
                new WaitCommand(0.5),
                new ReturnToIdle()
        );
    }

    @Override
    public Pose2d getInitialPose() {
        return FieldPositions.blueCoOp.getNodePoints()[2][1].toPose2d().plus(new Transform2d(new Translation2d(RobotPositions.scoringOffset, 0), Rotation2d.fromDegrees(180)));
    }

    @Override
    public State getInitialState() {
        return State.AUTO_SHOT;
    }
}
