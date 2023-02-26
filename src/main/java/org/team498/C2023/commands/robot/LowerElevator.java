package org.team498.C2023.commands.robot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import org.team498.C2023.commands.coneariser.SetConeARiserState;
import org.team498.C2023.commands.elevator.SetElevatorState;
import org.team498.C2023.commands.wrist.SetWristState;
import org.team498.C2023.subsystems.ConeARiser;
import org.team498.C2023.subsystems.Elevator;
import org.team498.C2023.subsystems.Wrist;

public class LowerElevator extends SequentialCommandGroup {
    public LowerElevator() {
        super(
                new SetWristState(Wrist.State.TRAVEL),
                new SetElevatorState(Elevator.State.IDLE),
                // new WaitCommand(3),
                new SetWristState(Wrist.State.IDLE));
    }
}
