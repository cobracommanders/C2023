package org.team498.C2023.commands.robot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FullScore extends SequentialCommandGroup{
    public FullScore(){
        super(
            new PrepareToScore(),
            new WaitCommand(.5),
            new Score()
        );
    }
}
