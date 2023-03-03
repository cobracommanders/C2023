package org.team498.C2023.commands.robot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.team498.C2023.commands.elevator.SetElevatorState;
import org.team498.C2023.commands.intake.SetIntake;
import org.team498.C2023.commands.manipulator.SetManipulatorState;
import org.team498.C2023.commands.wrist.SetWristState;
import org.team498.C2023.subsystems.Elevator;
import org.team498.C2023.subsystems.Wrist;
import org.team498.C2023.subsystems.Intake;
import org.team498.C2023.subsystems.Manipulator;


public class IntakeCommand extends SequentialCommandGroup {
    public IntakeCommand() {
        super(
                new SetWristState(Wrist.State.TRAVEL),
                new SetElevatorState(Elevator.State.IDLE),
                // new WaitCommand(3),
                new SetWristState(Wrist.State.INTAKE),
                new SetManipulatorState(Manipulator.State.COLLECT),
                new SetIntake(Intake.State.INTAKE)
        );
    }
}