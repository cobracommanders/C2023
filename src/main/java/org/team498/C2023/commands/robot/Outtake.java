package org.team498.C2023.commands.robot;

import org.team498.C2023.RobotState;
import org.team498.C2023.State;
import org.team498.C2023.commands.SetRobotState;
import org.team498.C2023.commands.elevatorWrist.SetElevatorWristState;
import org.team498.C2023.commands.intakeRollers.SetIntakeRollersState;
import org.team498.C2023.commands.intakeWrist.SetIntakeWristState;
import org.team498.C2023.commands.manipulator.SetManipulatorState;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Outtake extends SequentialCommandGroup{
    public Outtake(){
        super(
            new SetRobotState(State.OUTTAKE),
            new ParallelCommandGroup(
                new SetIntakeWristState(),
                new SetElevatorWristState(),
                new SetIntakeRollersState(),
                new SetManipulatorState()
            )
        );
    }
}
