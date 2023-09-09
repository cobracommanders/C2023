package org.team498.C2023.commands.intakeWrist;

import org.team498.C2023.RobotState;
import org.team498.C2023.State;
import org.team498.C2023.subsystems.IntakeWrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetIntakeWristState extends CommandBase {
    private State.IntakeWrist state;

    public SetIntakeWristState(){
        addRequirements(IntakeWrist.getInstance());
    }

    @Override
    public void initialize() {
        state = RobotState.getInstance().getState().intakeWrist;
        IntakeWrist.getInstance().setState(state);
    }

    @Override
    public boolean isFinished(){
        return true;//IntakeWrist.getInstance().atSetpoint();
    }

}
