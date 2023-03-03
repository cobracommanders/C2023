package org.team498.C2023.commands.intake;

import org.team498.C2023.commands.manipulator.SetManipulatorState;
import org.team498.C2023.commands.wrist.SetWristState;
import org.team498.C2023.subsystems.Intake;
import org.team498.C2023.subsystems.Manipulator;
import org.team498.C2023.subsystems.Wrist;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Shoot extends SequentialCommandGroup {
    public Shoot() {
        super(new ParallelCommandGroup(new SetWristState(Wrist.State.SPIT),
                new SetIntake(Intake.State.SPIT)),
                new WaitCommand(3),
                new SetManipulatorState(Manipulator.State.SCORE),
                new WaitCommand(1),
                new SetIntake(Intake.State.IDLE),
                new SetManipulatorState(Manipulator.State.IDLE));
    }
}
