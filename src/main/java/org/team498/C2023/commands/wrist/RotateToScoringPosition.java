package org.team498.C2023.commands.wrist;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team498.C2023.subsystems.Wrist;

public class RotateToScoringPosition extends CommandBase {
    private final Wrist wrist = Wrist.getInstance();

    @Override
    public void initialize() {
        wrist.setState(wrist.getNextScoringPosition());
    }

    @Override
    public boolean isFinished() {
        SmartDashboard.putBoolean("Wrist at setpoint", wrist.atSetpoint());
        return wrist.atSetpoint();
    }
}

