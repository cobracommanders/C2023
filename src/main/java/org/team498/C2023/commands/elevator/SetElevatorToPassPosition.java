package org.team498.C2023.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team498.C2023.subsystems.Elevator;

public class SetElevatorToPassPosition extends CommandBase {
    private final Elevator elevator = Elevator.getInstance();

    public SetElevatorToPassPosition() {
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setState(Elevator.State.CONEARISER);
    }

    @Override
    public boolean isFinished() {
        return elevator.atSetpoint();
    }
}
