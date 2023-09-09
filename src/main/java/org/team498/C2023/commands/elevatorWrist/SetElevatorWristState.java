package org.team498.C2023.commands.elevatorWrist;

import java.time.Period;

import org.team498.C2023.RobotState;
import org.team498.C2023.State;
import org.team498.C2023.subsystems.ElevatorWrist;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class SetElevatorWristState extends CommandBase{
    private State.ElevatorWrist state;

    public SetElevatorWristState(){
        addRequirements(ElevatorWrist.getInstance());
    }

    @Override
    public void initialize() {
        state = RobotState.getInstance().getState().elevatorWrist;
        ElevatorWrist.getInstance().setState(state);
    }

    @Override
    public boolean isFinished() {
        return true;//ElevatorWrist.getInstance().atSetpoint();
    }
}
