package org.team498.C2023.commands.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team498.C2023.subsystems.Wrist;

public class SetWristState extends CommandBase {
    private final Wrist wrist = Wrist.getInstance();
    private final Wrist.State state;

    public SetWristState(Wrist.State state) {
        this.state = state;
        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        wrist.setState(state);
    }

    @Override
    public boolean isFinished() {
        return wrist.atSetpoint();
    }

}
