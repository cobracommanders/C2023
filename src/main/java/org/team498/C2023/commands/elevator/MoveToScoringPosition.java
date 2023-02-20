package org.team498.C2023.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team498.C2023.subsystems.Elevator;

public class MoveToScoringPosition extends CommandBase {
    private final Elevator elevator = Elevator.getInstance();

    @Override
    public void initialize() {
        elevator.setState(elevator.getNextScoringPosition());
    }

    @Override
    public boolean isFinished() {
        return elevator.atSetpoint();
    }
}
