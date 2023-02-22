package org.team498.C2023.commands.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team498.C2023.subsystems.Wrist;

public class SetWristToPassPosition extends CommandBase {
    private final Wrist wrist = Wrist.getInstance();

    public SetWristToPassPosition() {
        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        wrist.setState(Wrist.State.CONEARISER);
    }

    @Override
    public boolean isFinished() {
        return wrist.atSetpoint();
    }
}

