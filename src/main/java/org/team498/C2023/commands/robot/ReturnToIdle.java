package org.team498.C2023.commands.robot;

import org.team498.C2023.subsystems.Manipulator;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import org.team498.C2023.RobotState;
import org.team498.C2023.State;
import org.team498.C2023.commands.SetRobotState;
import org.team498.C2023.commands.elevator.SetElevatorState;
import org.team498.C2023.commands.elevatorWrist.SetElevatorWristState;
import org.team498.C2023.commands.intakeRollers.SetIntakeRollersState;
import org.team498.C2023.commands.intakeWrist.SetIntakeWristState;
import org.team498.C2023.commands.manipulator.SetManipulatorState;
import org.team498.C2023.subsystems.IntakeWrist;

public class ReturnToIdle extends SequentialCommandGroup{
       public ReturnToIdle(){
        super(
            new SetRobotState(State.TRAVEL_CUBE),
            new ParallelCommandGroup(
                new SetIntakeWristState(),
                new SetElevatorWristState()
            ),
            new SetRobotState(State.IDLE_CUBE), 
            new ParallelCommandGroup(
                new SetElevatorState(),
                new SetManipulatorState(),
                new SetIntakeRollersState()
            ),
            new WaitCommand(2),
            new ParallelCommandGroup(
                new SetElevatorWristState(),
                new SetIntakeWristState()
            )
        );
       }
}
