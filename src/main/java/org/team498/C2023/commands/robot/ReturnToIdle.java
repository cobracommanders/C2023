package org.team498.C2023.commands.robot;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team498.C2023.RobotState;
import org.team498.C2023.State;
import org.team498.C2023.commands.SetRobotState;
import org.team498.C2023.commands.coneariser.SetConeARiserToNextState;
import org.team498.C2023.commands.elevator.SetElevatorToNextState;
import org.team498.C2023.commands.elevatorwrist.SetElevatorWristToNextState;
import org.team498.C2023.commands.intakerollers.SetIntakeRollersToNextState;
import org.team498.C2023.commands.intakewrist.SetIntakeWristToNextState;
import org.team498.C2023.commands.manipulator.SetManipulatorToNextState;

public class ReturnToIdle extends SequentialCommandGroup {
    public ReturnToIdle() {
        super(
                new ConditionalCommand(new SetRobotState(State.TRAVEL_CONE), new SetRobotState(State.TRAVEL_CUBE), () -> RobotState.getInstance().inConeMode()),
                new SetManipulatorToNextState(),
                new ParallelCommandGroup(
                        new SetElevatorWristToNextState(),
                        new SetIntakeWristToNextState()),
                new SetConeARiserToNextState(),
                new ConditionalCommand(new SetRobotState(State.IDLE_CONE), new SetRobotState(State.IDLE_CUBE), () -> RobotState.getInstance().inConeMode()),
                new SetElevatorToNextState(),
                new ParallelCommandGroup(
                        new SetElevatorWristToNextState(),
                        new SetIntakeWristToNextState()),
                new SetIntakeRollersToNextState(),
                new SetManipulatorToNextState(),
                new SetConeARiserToNextState());
    }

}
