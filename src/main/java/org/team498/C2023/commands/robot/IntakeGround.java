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

public class IntakeGround extends SequentialCommandGroup{
    public IntakeGround(){
        super(
            new SetRobotState(State.INTAKE),
            new ParallelCommandGroup(
                new SetIntakeWristState(State.IntakeWrist.INTAKE),
                new SetElevatorWristState(State.ElevatorWrist.INTAKE),
                new SetIntakeRollersState(State.IntakeRollers.INTAKE),
                new SetManipulatorState(State.Manipulator.INTAKE_CUBE)
            )
        );
    }
}
