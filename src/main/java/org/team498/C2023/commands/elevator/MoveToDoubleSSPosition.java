package org.team498.C2023.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team498.C2023.RobotState;
import org.team498.C2023.subsystems.Elevator;

public class MoveToDoubleSSPosition extends CommandBase {
    private final Elevator elevator = Elevator.getInstance();

    @Override
    public void initialize() {
        if (RobotState.getInstance().hasCone()) {
            elevator.setState(Elevator.State.DOUBLE_SS_CONE);
        } else {
            elevator.setState(Elevator.State.DOUBLE_SS_CUBE);
        }
    }

    @Override
    public void end(boolean interrupted) {
        elevator.atSetpoint();
    }
}
