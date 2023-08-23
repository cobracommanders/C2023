package org.team498.C2023;

import org.team498.C2023.Constants.OIConstants;
import org.team498.C2023.RobotState.GameMode;
import org.team498.C2023.RobotState.ScoringOption;
import org.team498.C2023.commands.drivetrain.DefenseDrive;
import org.team498.C2023.commands.drivetrain.TargetDrive;
import org.team498.C2023.commands.elevator.ManualElevator;
import org.team498.C2023.commands.elevator.SetElevatorState;
import org.team498.C2023.commands.elevatorWrist.ManualElevatorWrist;
import org.team498.C2023.commands.elevatorWrist.SetElevatorWristState;
import org.team498.C2023.commands.intakeWrist.SetIntakeWristState;
import org.team498.C2023.commands.robot.*;
import org.team498.C2023.subsystems.Drivetrain;
import org.team498.C2023.subsystems.ElevatorWrist;
import org.team498.C2023.subsystems.Manipulator;
import org.team498.lib.drivers.Gyro;
import org.team498.lib.drivers.Xbox;
import org.team498.lib.wpilib.ChoiceCommand;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class Controls {
    public final Xbox driver = new Xbox(OIConstants.DRIVER_CONTROLLER_ID);
    public final Xbox operator = new Xbox(OIConstants.OPERATOR_CONTROLLER_ID);

    private final RobotState robotState = RobotState.getInstance();

    public Controls() {
        driver.setDeadzone(0.15);
        driver.setTriggerThreshold(0.2);
        operator.setDeadzone(0.2);
        operator.setTriggerThreshold(0.2);
    }

    public void configureDefaultCommands() {
        Drivetrain.getInstance()
                  .setDefaultCommand(new DefenseDrive(driver::leftYSquared, driver::leftXSquared, driver::rightX, driver.rightBumper()));
    }

    public void configureDriverCommands() {
        driver.leftTrigger()
              .onTrue(new IntakeGround())
              .onFalse(new ReturnToIdle());
        driver.leftBumper()
              .onTrue(new Outtake())
              .onFalse(new ReturnToIdle());
        driver.A().onTrue(runOnce(() -> Gyro.getInstance().setYaw(0)));
        //driver.B().onTrue(new RealignCone());
        driver.rightTrigger()
              .onTrue(new PrepareToScore())
              .onFalse(
                sequence(
                    either(
                            waitSeconds(0),
                            waitSeconds(0.1),
                            () -> RobotState.getInstance().inConeMode()
                        ),
                        new Score()
                )
              );
        //driver.Y().onTrue(Robot.fullCheck.test());
        // driver.X().onTrue(new SetElevatorWristState(State.ElevatorWrist.SINGLE_SS)).onFalse(new SetElevatorWristState(State.ElevatorWrist.IDLE_CUBE));
        // driver.Y().onTrue(new SetIntakeWristState(State.IntakeWrist.INTAKE)).onFalse(new SetIntakeWristState(State.IntakeWrist.IDLE_IN));

       // driver.start().whileTrue(new FixCube()).onFalse(new ReturnToIdle());
    }

    public void configureOperatorCommands() {
        operator.Y().onTrue(runOnce(() -> robotState.setNextScoringOption(RobotState.ScoringOption.TOP)));
        operator.B().onTrue(runOnce(() -> robotState.setNextScoringOption(RobotState.ScoringOption.MID)));
        operator.A().onTrue(runOnce(() -> robotState.setNextScoringOption(RobotState.ScoringOption.LOW)));
        
        operator.rightBumper().onTrue(runOnce(() -> robotState.setCurrentGameMode(GameMode.CONE)));
        operator.leftBumper().onTrue(runOnce(() -> robotState.setCurrentGameMode(GameMode.CUBE)));

        operator.leftTrigger().onTrue(new IntakeSubStation()).onFalse(new ReturnToIdle());

        operator.start().toggleOnTrue(new ManualElevator(() -> -operator.rightY()));
        operator.back().toggleOnTrue(new ManualElevatorWrist(() -> -operator.leftY()));

        operator.POV90().whileTrue(runOnce(() -> Manipulator.getInstance().setState(State.Manipulator.INTAKE_CONE)));
        operator.POVMinus90().whileTrue(runOnce(() -> Manipulator.getInstance().setState(State.Manipulator.MID_CONE)));

        // operator.POV0().onTrue(runOnce(() -> ElevatorWrist.getInstance().incrementOffset(0.01)));
        // operator.POV180().onTrue(runOnce(() -> ElevatorWrist.getInstance().incrementOffset(-0.01)));
    }
}