package org.team498.C2023;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.team498.C2023.Constants.OIConstants;
import org.team498.C2023.RobotState.GamePiece;
import org.team498.C2023.commands.drivetrain.DefenseDrive;
import org.team498.C2023.commands.elevator.ManualElevator;
import org.team498.C2023.commands.elevatorwrist.ManualElevatorWrist;
import org.team498.C2023.commands.robot.*;
import org.team498.C2023.subsystems.Drivetrain;
import org.team498.lib.drivers.Gyro;
import org.team498.lib.drivers.Xbox;

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
        driver.rightTrigger().onTrue(new ConditionalCommand(new Score(), new Spit(), () -> robotState.getNextDriveteamState() != State.SPIT_CUBE));
        driver.leftTrigger().onTrue(new InstantCommand(() -> robotState.setState(State.GROUND_CUBE)).alongWith(new GroundIntake())).onFalse(new ReturnToIdle());
        driver.A().onTrue(new InstantCommand(() -> Gyro.getInstance().setYaw(0)));
        driver.X().onTrue(new InstantCommand(() -> robotState.setState(State.OUTTAKE)).alongWith(new GroundIntake())).onFalse(new ReturnToIdle());
    }

    public void configureOperatorCommands() {
        operator.Y().onTrue(new InstantCommand(() -> robotState.setNextDriveteamState(robotState.inConeMode() ? State.TOP_CONE : State.TOP_CUBE)));
        operator.B().onTrue(new InstantCommand(() -> robotState.setNextDriveteamState(robotState.inConeMode() ? State.MID_CONE : State.MID_CUBE)));
        operator.A().onTrue(new InstantCommand(() -> robotState.setNextDriveteamState(State.SPIT_CUBE)));

        operator.X().whileTrue(new FixCube()).onFalse(new ReturnToIdle());

        operator.rightBumper().onTrue(new InstantCommand(() -> robotState.setCurrentGameMode(GamePiece.CONE)));
        operator.leftBumper().onTrue(new InstantCommand(() -> robotState.setCurrentGameMode(GamePiece.CUBE)));

        operator.rightTrigger().onTrue(new InstantCommand(() -> robotState.setNextDriveteamState(State.DOUBLE_SS)).alongWith(new CollectFromSS())).onFalse(new ReturnToIdle());
        operator.leftTrigger().onTrue(new InstantCommand(() -> robotState.setNextDriveteamState(State.SINGLE_SS)).alongWith(new CollectFromSS())).onFalse(new ReturnToIdle());

        operator.start().toggleOnTrue(new ManualElevator(() -> -operator.leftY()));
        operator.back().toggleOnTrue(new ManualElevatorWrist(() -> -operator.rightY()));
    }
}