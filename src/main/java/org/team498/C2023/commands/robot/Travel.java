package org.team498.C2023.commands.robot;

import org.team498.C2023.State;
import org.team498.C2023.commands.SetRobotState;
import org.team498.C2023.commands.elevatorWrist.SetElevatorWristState;
import org.team498.C2023.commands.intakeWrist.SetIntakeWristState;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Travel extends SequentialCommandGroup{
    public Travel(){
        super(
            new SetRobotState(State.TRAVEL_CUBE),
            new ParallelCommandGroup(
                new SetElevatorWristState(),
                new SetIntakeWristState()
            )
        );
    }
}
