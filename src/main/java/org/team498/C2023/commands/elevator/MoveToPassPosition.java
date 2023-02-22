package org.team498.C2023.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team498.C2023.RobotState;
import org.team498.C2023.subsystems.Elevator;

public class MoveToPassPosition extends CommandBase {
    private final Elevator elevator = Elevator.getInstance();

    @Override
    public void initialize() {
        RobotState.getInstance().setNextScoringHeight(PASS_MODE);
        if (RobotState.getInstance().inConeMode()) {
            elevator.setState(Elevator.State.PASS_CONE);
        } else {
            elevator.setState(Elevator.State.PASS_CUBE);
        }
    }

    @Override
    public boolean isFinished() {
        return elevator.atSetpoint();
    }
}
