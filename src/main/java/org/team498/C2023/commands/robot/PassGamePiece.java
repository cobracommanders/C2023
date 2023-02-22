package org.team498.C2023.commands.robot;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.team498.C2023.commands.coneariser.PassFromConeARiser;
import org.team498.C2023.commands.coneariser.StopConeARiser;
import org.team498.C2023.commands.elevator.SetElevatorToPassPosition;
import org.team498.C2023.commands.manipulator.SetManipulatorState;
import org.team498.C2023.commands.manipulator.StopManipulator;
import org.team498.C2023.commands.wrist.SetWristToPassPosition;
import org.team498.C2023.subsystems.Manipulator;

public class PassGamePiece extends SequentialCommandGroup {
    public PassGamePiece() {
        super(new ParallelCommandGroup(
                new SetWristToPassPosition(),
                new SetElevatorToPassPosition()),
              new PassFromConeARiser(),
              new SetManipulatorState(Manipulator.State.COLLECT),
              new WaitCommand(1),
              new StopManipulator(),
              new StopConeARiser()
        );
    }
}
