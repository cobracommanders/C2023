package org.team498.C2023.commands.elevatorWrist;

import java.time.Period;

import org.team498.C2023.State;
import org.team498.C2023.subsystems.ElevatorWrist;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class SetElevatorWristState extends CommandBase{
    private final State.ElevatorWrist state;

    public SetElevatorWristState(State.ElevatorWrist state){
        addRequirements(ElevatorWrist.getInstance());
        this.state = state;
    }

    @Override
    public void initialize() {
        ElevatorWrist.getInstance().setState(state);
    }
}
