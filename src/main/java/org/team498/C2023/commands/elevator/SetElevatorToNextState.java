package org.team498.C2023.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.team498.C2023.RobotState;
import org.team498.C2023.subsystems.Elevator;

public class SetElevatorToNextState extends CommandBase {
    private final Elevator elevator = Elevator.getInstance();

    public SetElevatorToNextState() {
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setState(RobotState.getInstance().getCurrentState().elevator);
    }

    @Override
    public boolean isFinished() {
        return elevator.atSetpoint();
    }
}
