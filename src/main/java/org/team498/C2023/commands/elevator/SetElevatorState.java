package org.team498.C2023.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team498.C2023.subsystems.Elevator;

public class SetElevatorState extends CommandBase {
    private final Elevator elevator = Elevator.getInstance();
    private final Elevator.State state;

    public SetElevatorState(Elevator.State state) {
        this.state = state;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setState(state);
    }

    @Override
    public boolean isFinished() {
        return elevator.atSetpoint();
    }

}
