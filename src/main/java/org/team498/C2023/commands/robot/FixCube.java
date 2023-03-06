package org.team498.C2023.commands.robot;

import org.team498.C2023.commands.elevator.SetElevatorState;
import org.team498.C2023.commands.intake.SetIntakeState;
import org.team498.C2023.commands.manipulator.SetManipulatorState;
import org.team498.C2023.commands.wrist.SetWristState;
import org.team498.C2023.subsystems.Elevator;
import org.team498.C2023.subsystems.Intake;
import org.team498.C2023.subsystems.Manipulator;
import org.team498.C2023.subsystems.Wrist;

import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FixCube extends RepeatCommand {
    public FixCube() {
        super(
                new SequentialCommandGroup(
                        new SetManipulatorState(Manipulator.State.CONEARISER),
                        new SetIntakeState(Intake.State.IDLE_OUT),
                        new SetWristState(Wrist.State.TRAVEL),
                        new SetElevatorState(Elevator.State.UNSTICK_CUBE),
                        new SetWristState(Wrist.State.UNSTICK_CUBE),
                        new SetElevatorState(Elevator.State.IDLE),
                        new SetIntakeState(Intake.State.IDLE_IN),
                        new WaitCommand(0.5)));
    }
}
