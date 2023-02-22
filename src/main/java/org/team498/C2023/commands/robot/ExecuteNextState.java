package org.team498.C2023.commands.robot;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.team498.C2023.commands.elevator.SetElevatorToNextState;
import org.team498.C2023.commands.elevator.SetElevatorState;
import org.team498.C2023.commands.manipulator.SetManipulatorState;
import org.team498.C2023.commands.manipulator.SetManipulatorToNextState;
import org.team498.C2023.commands.manipulator.StopManipulator;
import org.team498.C2023.commands.wrist.SetWristToNextState;
import org.team498.C2023.subsystems.Elevator;
import org.team498.C2023.subsystems.Manipulator;

public class ExecuteNextState extends SequentialCommandGroup {
    public ExecuteNextState() {
        super(new ParallelCommandGroup(
                new SetElevatorToNextState(),
                new SetWristToNextState()),
              new WaitCommand(0.5),
              new SetManipulatorToNextState(),
              new WaitCommand(0.5),
              new StopManipulator(),
              new SetElevatorState(Elevator.State.IDLE),
              new SetManipulatorState(Manipulator.State.IDLE)
        );
    }
}
