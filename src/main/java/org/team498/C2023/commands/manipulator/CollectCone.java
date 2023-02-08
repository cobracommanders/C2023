package org.team498.C2023.commands.manipulator;

import org.team498.C2023.subsystems.Manipulator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class CollectCone extends SequentialCommandGroup {
    public CollectCone() {
        Manipulator manipulator = Manipulator.getInstance();
        addCommands(manipulator.setOuttake(Manipulator.State.INTAKE_CONE),
                    new WaitCommand(0.4),
                    new WaitUntilCommand(manipulator::isStalling),
                    manipulator.stop());
    }
}