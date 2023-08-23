package org.team498.C2023.commands.robot;

import org.team498.C2023.RobotState;
import org.team498.C2023.State;
import org.team498.C2023.commands.SetRobotState;
import org.team498.C2023.commands.elevator.SetElevatorState;
import org.team498.C2023.commands.manipulator.SetManipulatorState;

import com.ctre.phoenix.schedulers.SequentialScheduler;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Score extends SequentialCommandGroup{
    public Score(){
        super(
            new SetManipulatorState(),
            new WaitCommand(0.1),
            new ReturnToIdle()
        );
    }
}
