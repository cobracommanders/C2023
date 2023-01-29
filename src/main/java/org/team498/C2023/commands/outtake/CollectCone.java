package org.team498.C2023.commands.outtake;

import org.team498.C2023.subsystems.Outtake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class CollectCone extends SequentialCommandGroup {

    private final static Outtake outtake = Outtake.getInstance();

    public CollectCone() {
        super(outtake.setOuttake(Outtake.State.INTAKE_CONE),
                new WaitCommand(0.4),
                new WaitUntilCommand(() -> outtake.isBottomStalling()),
                outtake.setOuttake(Outtake.State.IDLE));
    }
}