package org.team498.C2023;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import org.team498.C2023.Constants.OIConstants;
import org.team498.C2023.Ports.Elevator;
import org.team498.C2023.RobotState.GameMode;
import org.team498.C2023.commands.drivetrain.AutoEngage;
import org.team498.C2023.commands.drivetrain.AutoEngage2;
import org.team498.C2023.commands.drivetrain.DefenseDrive;
import org.team498.C2023.commands.drivetrain.DriveToTipAndBalance;
import org.team498.C2023.commands.elevator.ManualElevator;
import org.team498.C2023.commands.elevator.SetElevatorToNextState;
import org.team498.C2023.commands.elevatorwrist.ManualElevatorWrist;
import org.team498.C2023.commands.elevatorwrist.SetElevatorWristToNextState;
import org.team498.C2023.commands.intakerollers.SetIntakeRollersToNextState;
import org.team498.C2023.commands.intakewrist.SetIntakeWristToNextState;
import org.team498.C2023.commands.robot.*;
import org.team498.C2023.subsystems.Drivetrain;
import org.team498.C2023.subsystems.ElevatorWrist;
import org.team498.C2023.subsystems.Manipulator;
import org.team498.lib.drivers.Gyro;
import org.team498.lib.drivers.Xbox;
import org.team498.lib.wpilib.ChoiceCommand;

public class Controls {
    public final Xbox driver = new Xbox(OIConstants.DRIVER_CONTROLLER_ID);
    public final Xbox operator = new Xbox(OIConstants.OPERATOR_CONTROLLER_ID);

    private final RobotState robotState = RobotState.getInstance();

    public Controls() {
        driver.setDeadzone(0.2);
        driver.setTriggerThreshold(0.2);
        operator.setDeadzone(0.2);
        operator.setTriggerThreshold(0.2);
    }

    public void configureDefaultCommands() {
        Drivetrain.getInstance().setDefaultCommand(new DefenseDrive(driver::leftYSquared, driver::leftXSquared, driver::rightX, driver.rightBumper()));
    }

    public void configureDriverCommands() {
        driver.leftTrigger().onTrue(new InstantCommand(() -> robotState.setState(State.GROUND_CUBE)).andThen(new GroundIntake())).onFalse(new ReturnToIdle());
        driver.leftBumper().onTrue(new InstantCommand(() -> robotState.setState(State.OUTTAKE)).andThen(new GroundIntake())).onFalse(new ReturnToIdle());
        driver.A().onTrue(new InstantCommand(() -> Gyro.getInstance().setYaw(0)));

        driver.B().onTrue(new RealignCone());


        driver.rightTrigger()
                // .whileTrue(new ConditionalCommand(new TargetDrive(driver::leftYSquared, driver::leftXSquared, driver.rightBumper(), () -> RobotPositions.getNextScoringNodePosition().transformBy(Drivetrain.getInstance().getVelocity().times(5).inverse())), Commands.none(), () -> RobotState.getInstance().inCubeMode() && RobotState.getInstance().getNextScoringOption() != ScoringOption.SPIT))
                .onTrue(new PrepareToScore())
                .onFalse(new ChoiceCommand(() -> {
                    Command command = switch (robotState.getNextScoringOption()) {
                        case TOP, MID -> new SequentialCommandGroup(new VerifyScoreLocation(), new ConditionalCommand(new WaitCommand(0.75), new WaitCommand(0.1), () -> RobotState.getInstance().inConeMode()), new Score());
                        case SPIT -> new Spit();
                    };
                    return command;
                }
            ));

        driver.back().onTrue(new InstantCommand(() -> ElevatorWrist.getInstance().setEnabled(!ElevatorWrist.getInstance().getEnabled())));
        driver.start().onTrue(new InstantCommand(() -> {
            ElevatorWrist.getInstance().PID.reset();
            ElevatorWrist.getInstance().PID.setSetpoint(0);
        }));

        driver.X().onTrue(new Spit());

    }

    /**
     * 
     */
    public void configureOperatorCommands() {
        operator.Y().onTrue(new InstantCommand(() -> robotState.setNextScoringOption(RobotState.ScoringOption.TOP)));
        operator.B().onTrue(new InstantCommand(() -> robotState.setNextScoringOption(RobotState.ScoringOption.MID)));
        operator.A().onTrue(new InstantCommand(() -> robotState.setNextScoringOption(RobotState.ScoringOption.SPIT)));
        operator.X().toggleOnTrue(new StartEndCommand(() -> robotState.setShootDrive(true), () -> robotState.setShootDrive(false)));

        operator.rightBumper().onTrue(new InstantCommand(() -> robotState.setCurrentGameMode(GameMode.CONE)));
        operator.leftBumper().onTrue(new InstantCommand(() -> robotState.setCurrentGameMode(GameMode.CUBE)));

        operator.leftTrigger().onTrue(new CollectFromSS()).onFalse(new ReturnToIdle());

        operator.POV180().whileTrue(new FixCube()).onFalse(new ReturnToIdle());
        operator.rightTrigger().toggleOnTrue(new ConditionalCommand(new InstantCommand(()-> robotState.setState(State.SPIT_CUBE)), new InstantCommand(()-> robotState.setState(State.IDLE_CUBE)), ()-> robotState.getCurrentState() == State.IDLE_CUBE).andThen(new ParallelCommandGroup(
            new SetElevatorWristToNextState(),
            new SetIntakeWristToNextState(),
            new SetElevatorToNextState(),
            new SetIntakeRollersToNextState())));

        operator.start().toggleOnTrue(new ManualElevator(() -> -operator.rightY()));
        operator.back().toggleOnTrue(new ManualElevatorWrist(() -> -operator.leftY()));

        operator.POV90().whileTrue(new InstantCommand(()-> Manipulator.getInstance().setState(State.Manipulator.INTAKE_CONE)));
        operator.POVMinus90().whileTrue(new InstantCommand(()-> Manipulator.getInstance().setState(State.Manipulator.MID_CONE)));

        operator.POV0().onTrue(new RealignCone());
    }
}