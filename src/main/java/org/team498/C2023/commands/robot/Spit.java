package org.team498.C2023.commands.robot;

import org.team498.C2023.State;
import org.team498.C2023.commands.SetRobotState;
import org.team498.C2023.commands.elevator.SetElevatorToNextState;
import org.team498.C2023.commands.elevatorwrist.SetElevatorWristToNextState;
import org.team498.C2023.commands.intakerollers.SetIntakeRollersToNextState;
import org.team498.C2023.commands.intakewrist.SetIntakeWristToNextState;
import org.team498.C2023.commands.manipulator.SetManipulatorToNextState;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Spit extends SequentialCommandGroup {
    public Spit() {
        super(
                new SetRobotState(State.SPIT),
                new ParallelCommandGroup(
                        new SetElevatorWristToNextState(),
                        new SetIntakeWristToNextState(),
                        new SetIntakeRollersToNextState(),
                        new SetElevatorToNextState()),
                new WaitCommand(1), //TODO make this faster
                new SetManipulatorToNextState(),
                new WaitCommand(1),
                new ReturnToIdle());
    }
}
