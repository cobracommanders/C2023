package org.team498.C2023.commands.robot;

import org.team498.C2023.RobotState;
import org.team498.C2023.State;
import org.team498.C2023.commands.SetRobotState;
import org.team498.C2023.commands.elevator.SetElevatorToNextState;
import org.team498.C2023.commands.intakewrist.SetIntakeWristToNextState;
import org.team498.C2023.commands.manipulator.SetManipulatorToNextState;
import org.team498.C2023.commands.elevatorwrist.SetElevatorWristToNextState;
import org.team498.C2023.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class CollectFromSS extends SequentialCommandGroup {
    public CollectFromSS() {
        super(
                new SetRobotState(State.TRAVEL_EMPTY),
                new SetElevatorWristToNextState(),
                new SetIntakeWristToNextState(),
                new SetRobotState(RobotState.getInstance().getNextDriveteamState()),
                new ParallelCommandGroup(
                        new SetElevatorToNextState(),
                        new SetManipulatorToNextState(),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> Elevator.getInstance().aboveIntakeHeight() || Elevator.getInstance().atSetpoint()),
                                new ParallelCommandGroup(
                                        new SetElevatorWristToNextState(),
                                        new SetIntakeWristToNextState()))));
    }
}
