package org.team498.C2023.commands.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import org.team498.C2023.Robot;
import org.team498.C2023.RobotState;
import org.team498.C2023.commands.coneariser.SetConeARiserState;
import org.team498.C2023.commands.coneariser.SetConeARiserToNextState;
import org.team498.C2023.commands.drivetrain.DefenseDrive;
import org.team498.C2023.commands.drivetrain.LockWheels;
import org.team498.C2023.commands.elevator.SetElevatorToNextState;
import org.team498.C2023.commands.intake.SetIntakeState;
import org.team498.C2023.commands.manipulator.SetManipulatorState;
import org.team498.C2023.commands.manipulator.SetManipulatorToNextState;
import org.team498.C2023.commands.wrist.SetWristState;
import org.team498.C2023.commands.wrist.SetWristToNextState;
import org.team498.C2023.subsystems.ConeARiser;
import org.team498.C2023.subsystems.Elevator;
import org.team498.C2023.subsystems.Intake;
import org.team498.C2023.subsystems.Manipulator;
import org.team498.C2023.subsystems.Wrist;

public class Score extends SequentialCommandGroup {
    public Score() {
        super(
                new LockWheels(),
                new SetConeARiserToNextState(),
                new SetIntakeState(Intake.State.TRAVEL),
                new WaitCommand(0.2),
                new SetWristState(Wrist.State.TRAVEL),
                new ConditionalCommand(new SetManipulatorState(Manipulator.State.HOLD), Commands.none(), () -> RobotState.getInstance().inConeMode()),
                new ParallelRaceGroup(
                        new SetElevatorToNextState(),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> Elevator.getInstance().aboveIntakeHeight()),
                                new SetWristToNextState())),
                new SetWristToNextState(),
                new SetConeARiserState(ConeARiser.State.IDLE),
                new ConditionalCommand(new WaitCommand(1), new WaitCommand(0.5), () -> RobotState.getInstance().inConeMode()),
                new SetManipulatorToNextState(),
                new WaitCommand(0.5),
                new ParallelRaceGroup(
                        new DefenseDrive(
                                Robot.controls.driver::leftYSquared,
                                Robot.controls.driver::leftXSquared,
                                Robot.controls.driver::rightX,
                                Robot.controls.driver.rightBumper()),
                        new SequentialCommandGroup(
                                new LowerElevator(),
                                new SetManipulatorState(Manipulator.State.IDLE),
                                new SetIntakeState(Intake.State.IDLE_IN))));
    }
}
