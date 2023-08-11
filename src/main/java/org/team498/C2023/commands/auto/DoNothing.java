package org.team498.C2023.commands.auto;

import org.team498.lib.auto.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class DoNothing implements Auto {

    @Override
    public Command getCommand() {
        return new WaitCommand(2);
    }

    @Override
    public Pose2d getInitialPose() {
        return new Pose2d();
    }
    
}
