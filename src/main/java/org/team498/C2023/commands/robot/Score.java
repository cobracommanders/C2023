package org.team498.C2023.commands.robot;

import edu.wpi.first.wpilibj2.command.*;

import org.team498.C2023.RobotState;
import org.team498.C2023.State;
import org.team498.C2023.commands.SetRobotState;
import org.team498.C2023.commands.SetRobotToDriveteamState;
import org.team498.C2023.commands.coneariser.SetConeARiserToNextState;
import org.team498.C2023.commands.elevator.SetElevatorToNextState;
import org.team498.C2023.commands.intakewrist.SetIntakeWristToNextState;
import org.team498.C2023.commands.manipulator.SetManipulatorToNextState;
import org.team498.C2023.commands.elevatorwrist.SetElevatorWristToNextState;
import org.team498.C2023.subsystems.Elevator;

public class Score extends SequentialCommandGroup {
    public Score() {
        super(
                new ConditionalCommand(new SetRobotState(State.TRAVEL_CONE), new SetRobotState(State.TRAVEL_CUBE), () -> RobotState.getInstance().inConeMode()),
                new SetConeARiserToNextState(),
                new SetManipulatorToNextState(),
                new ParallelCommandGroup(
                        new SetIntakeWristToNextState(),
                        new SetElevatorWristToNextState()
                ),
                new SetRobotToDriveteamState(),
                new ParallelCommandGroup(
                        new SetElevatorToNextState(),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> Elevator.getInstance().aboveIntakeHeight() || Elevator.getInstance().atSetpoint()),
                                new ParallelCommandGroup(
                                        new SetElevatorWristToNextState(),
                                        new SetIntakeWristToNextState()))),
                new ConditionalCommand(new WaitCommand(0.75), new WaitCommand(0.1), () -> RobotState.getInstance().inConeMode()), //TODO make faster
                new SetManipulatorToNextState(),
                new WaitCommand(0.5), //TODO make faster

                new ReturnToIdle());
    }
}
