package org.team498.C2023.commands.robot;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.team498.C2023.Robot;
import org.team498.C2023.commands.drivetrain.OffenseDrive;
import org.team498.C2023.commands.elevator.MoveToDoubleSSPosition;
import org.team498.C2023.commands.elevator.SetElevatorState;
import org.team498.C2023.commands.manipulator.CollectGamePiece;
import org.team498.C2023.commands.manipulator.SetManipulatorState;
import org.team498.C2023.commands.manipulator.StopManipulator;
import org.team498.C2023.commands.wrist.RotateToDoubleSSPosition;
import org.team498.C2023.subsystems.Elevator;
import org.team498.C2023.subsystems.Manipulator;

public class    RetrieveFromDoubleSS extends SequentialCommandGroup {
    public RetrieveFromDoubleSS() {
        super(
                new ParallelCommandGroup(
                        new MoveToDoubleSSPosition(),
                        new RotateToDoubleSSPosition()//,
                        // new OffenseDrive(() -> 0, () -> 0, () -> Robot.rotationOffset, () -> false)
                ),
                new CollectGamePiece(),
                new WaitCommand(0.2),
                new StopManipulator()//,
                // new SetElevatorState(Elevator.State.IDLE),
                // new SetManipulatorState(Manipulator.State.IDLE)
        );
    }
}
