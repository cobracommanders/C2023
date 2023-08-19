package org.team498.C2023.commands.robot;

import org.team498.C2023.RobotState;
import org.team498.C2023.State;
import org.team498.C2023.Constants.ElevatorWrist;
import org.team498.C2023.commands.SetRobotState;
import org.team498.C2023.commands.elevator.SetElevatorState;
import org.team498.C2023.commands.elevatorWrist.SetElevatorWristState;
import org.team498.C2023.commands.intakeWrist.SetIntakeWristState;
import org.team498.C2023.commands.manipulator.SetManipulatorState;
import org.team498.C2023.subsystems.Manipulator;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PrepareToScore extends SequentialCommandGroup{
    public PrepareToScore(){
        super(
            new SetRobotState(State.TRAVEL_CUBE),
            new ParallelCommandGroup(
                new SetIntakeWristState(RobotState.getInstance().getState().intakeWrist),
                new SetElevatorWristState(RobotState.getInstance().getState().elevatorWrist)
            ),
            new SetRobotState(RobotState.getInstance().getNextScoringState()),
            new SetElevatorState(RobotState.getInstance().getState().elevator),
            new ParallelCommandGroup(
                new SetIntakeWristState(RobotState.getInstance().getState().intakeWrist),
                new SetElevatorWristState(RobotState.getInstance().getState().elevatorWrist)
            )
        );
    }
}
