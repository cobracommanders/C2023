package org.team498.C2023.commands.robot;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.team498.C2023.commands.elevator.SetElevatorState;
import org.team498.C2023.commands.wrist.SetWristState;
import org.team498.C2023.subsystems.Elevator;
import org.team498.C2023.subsystems.Wrist;

public class LowerElevator extends ParallelCommandGroup {
    public LowerElevator() {
        super(new SetElevatorState(Elevator.State.IDLE),
              new SetWristState(Wrist.State.IDLE));
    }
}
