package org.team498.C2023.commands.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.function.Supplier;

import org.team498.C2023.commands.drivetrain.DriveToPosition;
import org.team498.C2023.commands.elevator.MoveToDoubleSSPosition;
import org.team498.C2023.commands.elevator.SetElevatorState;
import org.team498.C2023.commands.manipulator.CollectGamePiece;
import org.team498.C2023.commands.manipulator.SetManipulatorState;
import org.team498.C2023.commands.manipulator.StopManipulator;
import org.team498.C2023.commands.wrist.RotateToDoubleSSPosition;
import org.team498.C2023.subsystems.Elevator;
import org.team498.C2023.subsystems.Manipulator;

public class CollectFromDoubleSS extends SequentialCommandGroup {
    public CollectFromDoubleSS(Supplier<Pose2d> target) {
        super(
                new ParallelCommandGroup(
                        new MoveToDoubleSSPosition(),
                        new RotateToDoubleSSPosition(),
                        new DriveToPosition(target),
                        new CollectGamePiece()),
                new WaitCommand(0.5),
                new StopManipulator(),
                new SetElevatorState(Elevator.State.IDLE),
                new SetManipulatorState(Manipulator.State.IDLE));
    }
}
