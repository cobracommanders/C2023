package org.team498.C2023.commands.robot;

import org.team498.C2023.commands.coneariser.SetConeARiserToNextState;
import org.team498.C2023.commands.elevator.SetElevatorToNextState;
import org.team498.C2023.commands.wrist.SetWristState;
import org.team498.C2023.commands.wrist.SetWristToNextState;
import org.team498.C2023.subsystems.Wrist;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RaiseElevatorToPosition extends SequentialCommandGroup {
    public RaiseElevatorToPosition() {
        super(new SetWristState(Wrist.State.TRAVEL),
                new SetElevatorToNextState(),
                new SetWristToNextState()
                );
    }
}
