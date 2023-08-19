package org.team498.C2023.commands.intakeWrist;

import org.team498.C2023.State;
import org.team498.C2023.subsystems.IntakeWrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetIntakeWristState extends CommandBase {
    private final State.IntakeWrist state;

    public SetIntakeWristState(State.IntakeWrist state){
        addRequirements(IntakeWrist.getInstance());
        this.state = state;
    }

    @Override
    public void initialize() {
        IntakeWrist.getInstance().setState(state);
    }

    @Override
    public boolean isFinished(){
        return IntakeWrist.getInstance().atSetpoint();
    }

}
