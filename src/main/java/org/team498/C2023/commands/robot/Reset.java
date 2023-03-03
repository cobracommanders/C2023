package org.team498.C2023.commands.robot;

import org.team498.C2023.commands.intake.SetIntake;
import org.team498.C2023.commands.manipulator.SetManipulatorState;
import org.team498.C2023.commands.manipulator.StopManipulator;
import org.team498.C2023.subsystems.Intake;
import org.team498.C2023.subsystems.Manipulator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Reset extends SequentialCommandGroup {
    public Reset() {
        super(
                new StopManipulator(),
                new LowerElevator(),
                new SetManipulatorState(Manipulator.State.IDLE),
                new SetIntake(Intake.State.IDLE)
        );
    }

}
