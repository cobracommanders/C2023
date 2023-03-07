package org.team498.C2023;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import org.team498.C2023.Constants.OIConstants;
import org.team498.C2023.RobotState.GamePiece;
import org.team498.C2023.commands.drivetrain.*;
import org.team498.C2023.commands.elevator.ManualElevator;
import org.team498.C2023.commands.robot.CollectFromSS;
import org.team498.C2023.commands.robot.FixCube;
import org.team498.C2023.commands.robot.GroundIntake;
import org.team498.C2023.commands.robot.ReturnToIdle;
import org.team498.C2023.commands.robot.Score;
import org.team498.C2023.commands.robot.Spit;
import org.team498.C2023.commands.elevatorwrist.ManualElevatorWrist;
import org.team498.C2023.subsystems.*;
import org.team498.lib.drivers.Gyro;
import org.team498.lib.drivers.Xbox;

public class Controls {
    public final Xbox driver = new Xbox(OIConstants.DRIVER_CONTROLLER_ID);
    public final Xbox operator = new Xbox(OIConstants.OPERATOR_CONTROLLER_ID);

    private final Drivetrain drivetrain = Drivetrain.getInstance();
    private final Elevator elevator = Elevator.getInstance();
    private final Manipulator manipulator = Manipulator.getInstance();
    private final ElevatorWrist elevatorWrist = ElevatorWrist.getInstance();
    private final IntakeWrist intakeWrist = IntakeWrist.getInstance();
    private final IntakeRollers intake = IntakeRollers.getInstance();
    private final ConeARiser coneARiser = ConeARiser.getInstance();
    private final RobotState robotState = RobotState.getInstance();

    public Controls() {
        driver.setDeadzone(0.2);
        driver.setTriggerThreshold(0.2);
        operator.setDeadzone(0.2);
        operator.setTriggerThreshold(0.2);
    }

    public void configureDefaultCommands() {
        drivetrain.setDefaultCommand(
                new DefenseDrive(driver::leftYSquared, driver::leftXSquared, driver::rightX, driver.rightBumper()));
    }

    public void configureDriverCommands() {
        driver.rightTrigger().onTrue(new Score());
        driver.leftBumper().onTrue(new Spit());
        driver.leftTrigger().onTrue(new InstantCommand(() -> robotState.setState(State.GROUND_CUBE)).alongWith(new GroundIntake())).onFalse(new ReturnToIdle());
        driver.A().onTrue(new InstantCommand(() -> Gyro.getInstance().setYaw(0)));
        driver.X().onTrue(new InstantCommand(() -> robotState.setState(State.OUTTAKE)).alongWith(new GroundIntake())).onFalse(new ReturnToIdle());
    }

    public void configureOperatorCommands() {
        operator.Y().onTrue(new InstantCommand(() -> robotState.setNextDriveteamState(robotState.inConeMode() ? State.TOP_CONE : State.TOP_CUBE)));
        operator.B().onTrue(new InstantCommand(() -> robotState.setNextDriveteamState(robotState.inConeMode() ? State.MID_CONE : State.MID_CUBE)));

        operator.X().whileTrue(new FixCube()).onFalse(new ReturnToIdle());

        operator.rightBumper().onTrue(new InstantCommand(() -> robotState.setCurrentGameMode(GamePiece.CONE)));
        operator.leftBumper().onTrue(new InstantCommand(() -> robotState.setCurrentGameMode(GamePiece.CUBE)));

        operator.rightTrigger().onTrue(new InstantCommand(() -> robotState.setNextDriveteamState(State.DOUBLE_SS)).alongWith(new CollectFromSS())).onFalse(new ReturnToIdle());
        operator.leftTrigger().onTrue(new InstantCommand(() -> robotState.setNextDriveteamState(State.SINGLE_SS)).alongWith(new CollectFromSS())).onFalse(new ReturnToIdle());

        operator.start().toggleOnTrue(new ManualElevator(() -> -operator.leftY()));
        operator.back().toggleOnTrue(new ManualElevatorWrist(() -> -operator.rightY()));
    }
}