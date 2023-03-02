package org.team498.C2023.commands.auto;

import org.team498.C2023.Robot;
import org.team498.C2023.RobotState;
import org.team498.C2023.RobotState.GamePiece;
import org.team498.C2023.RobotState.Height;
import org.team498.C2023.commands.drivetrain.BangBangBalance;
import org.team498.C2023.commands.elevator.SetElevatorState;
import org.team498.C2023.commands.manipulator.SetManipulatorState;
import org.team498.C2023.commands.robot.MoveToNextState;
import org.team498.C2023.commands.wrist.SetWristState;
import org.team498.C2023.subsystems.Drivetrain;
import org.team498.C2023.subsystems.Manipulator;
import org.team498.C2023.subsystems.Wrist;
import org.team498.C2023.subsystems.Elevator;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class CubeEngage extends SequentialCommandGroup {
    private final RobotState robot = RobotState.getInstance();
    public CubeEngage() {
        super(
            new ParallelDeadlineGroup(new WaitCommand(3),
                new InstantCommand(()-> RobotState.getInstance().setCurrentGameMode(GamePiece.CUBE)),
                new InstantCommand(()-> RobotState.getInstance().setNextHeight(Height.TOP)),
                new MoveToNextState()
            ),
            new SequentialCommandGroup(
                                    new WaitCommand(0.25),
                                    new SetManipulatorState(Manipulator.State.IDLE),
                                    new SetWristState(Wrist.State.IDLE),
                                    new SetElevatorState(Elevator.State.IDLE)
            ),
            new BangBangBalance()
        );
    }
}
