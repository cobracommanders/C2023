package org.team498.C2023;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import org.team498.C2023.Constants.OIConstants;
import org.team498.C2023.RobotState.GameMode;
import org.team498.C2023.commands.drivetrain.DefenseDrive;
import org.team498.C2023.commands.drivetrain.DriveToPosition;
import org.team498.C2023.commands.drivetrain.TargetDrive;
import org.team498.C2023.commands.elevator.ManualElevator;
import org.team498.C2023.commands.elevatorwrist.ManualElevatorWrist;
import org.team498.C2023.commands.robot.*;
import org.team498.C2023.subsystems.Drivetrain;
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
        driver.A().onTrue(new InstantCommand(() -> Gyro.getInstance().setYaw(0)));
        driver.X().onTrue(new InstantCommand(() -> robotState.setState(State.OUTTAKE)).alongWith(new GroundIntake())).onFalse(new ReturnToIdle());


        driver.rightTrigger()
                .whileTrue(new TargetDrive(driver::leftYSquared, driver::leftXSquared, driver.rightBumper(), RobotPositions::getNextScoringNodePosition))
                .onTrue(new PrepareToScore())
                .onFalse(new ChoiceCommand(() -> {
                    Command command = switch (robotState.getNextScoringOption()) {
                        case TOP, MID -> new Score();
                        case SPIT -> new Spit();
                    };
                    return command;
                }
            ));


        driver.start().onTrue(new DriveToPosition(() -> new Pose2d(8, 4, Rotation2d.fromDegrees(180))));

    }

    public void configureOperatorCommands() {
        operator.Y().onTrue(new InstantCommand(() -> robotState.setNextScoringOption(RobotState.ScoringOption.TOP)));
        operator.B().onTrue(new InstantCommand(() -> robotState.setNextScoringOption(RobotState.ScoringOption.MID)));
        operator.A().onTrue(new InstantCommand(() -> robotState.setNextScoringOption(RobotState.ScoringOption.SPIT)));
        operator.X().toggleOnTrue(new StartEndCommand(() -> robotState.setShootDrive(true), () -> robotState.setShootDrive(false)));

        operator.rightBumper().onTrue(new InstantCommand(() -> robotState.setCurrentGameMode(GameMode.CONE)));
        operator.leftBumper().onTrue(new InstantCommand(() -> robotState.setCurrentGameMode(GameMode.CUBE)));

        operator.leftTrigger().onTrue(new CollectFromSS()).onFalse(new ReturnToIdle());

        operator.rightTrigger().whileTrue(new FixCube()).onFalse(new ReturnToIdle());

        operator.start().toggleOnTrue(new ManualElevator(() -> -operator.leftY()));
        operator.back().toggleOnTrue(new ManualElevatorWrist(() -> -operator.rightY()));
    }
}