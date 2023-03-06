package org.team498.C2023.commands.robot;

import org.team498.C2023.commands.intake.SetIntakeState;
import org.team498.C2023.commands.manipulator.SetManipulatorState;
import org.team498.C2023.commands.wrist.SetWristState;
import org.team498.C2023.subsystems.Intake;
import org.team498.C2023.subsystems.Manipulator;
import org.team498.C2023.subsystems.Wrist;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Spit extends SequentialCommandGroup {
    public Spit() {
        super(
                new ParallelCommandGroup(
                        new SetWristState(Wrist.State.SPIT),
                        new SetIntakeState(Intake.State.SPIT)),
                new WaitCommand(1),
                new SetManipulatorState(Manipulator.State.SPIT),
                new WaitCommand(1),
                new SetIntakeState(Intake.State.IDLE_IN),
                new SetManipulatorState(Manipulator.State.IDLE));
    }
}
