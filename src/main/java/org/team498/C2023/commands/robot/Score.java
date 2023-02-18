package org.team498.C2023.commands.robot;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.team498.C2023.Robot;
import org.team498.C2023.commands.drivetrain.OffenseDrive;
import org.team498.C2023.commands.elevator.MoveToScoringPosition;
import org.team498.C2023.commands.elevator.SetElevatorState;
import org.team498.C2023.commands.manipulator.ScoreGamePiece;
import org.team498.C2023.commands.manipulator.SetManipulatorState;
import org.team498.C2023.commands.manipulator.StopManipulator;
import org.team498.C2023.commands.wrist.RotateToScoringPosition;
import org.team498.C2023.subsystems.Elevator;
import org.team498.C2023.subsystems.Manipulator;

public class Score extends SequentialCommandGroup {
    public Score() {
        super(
                new ParallelCommandGroup(
                        new MoveToScoringPosition(),
                        new RotateToScoringPosition(),
                        new OffenseDrive(() -> 0, () -> 0, () -> 180 - Robot.rotationOffset, () -> false)
                ),
                new WaitCommand(0.1),
                new ScoreGamePiece(),
                new WaitCommand(0.2),
                new StopManipulator(),
                new SetElevatorState(Elevator.State.IDLE),
                new SetManipulatorState(Manipulator.State.IDLE)

        );
    }
}
