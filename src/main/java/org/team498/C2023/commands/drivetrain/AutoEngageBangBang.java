package org.team498.C2023.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoEngageBangBang extends SequentialCommandGroup {
    public AutoEngageBangBang() {
        super(
            new AutoEngage2(),
            new BangBangBalance()
        );
    }
}
