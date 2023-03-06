package org.team498.C2023;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;

import org.team498.C2023.Constants.OIConstants;
import org.team498.C2023.RobotState.GamePiece;
import org.team498.C2023.RobotState.Height;
import org.team498.C2023.commands.auto.CubeEngage;
import org.team498.C2023.commands.drivetrain.*;
import org.team498.C2023.commands.elevator.ManualElevator;
import org.team498.C2023.commands.intake.SetIntakeState;
import org.team498.C2023.commands.manipulator.SetManipulatorState;
import org.team498.C2023.commands.manipulator.StopManipulator;
import org.team498.C2023.commands.robot.LowerElevator;
import org.team498.C2023.commands.robot.CollectFromSS;
import org.team498.C2023.commands.robot.FixCube;
import org.team498.C2023.commands.robot.IntakeCommand;
import org.team498.C2023.commands.robot.Reset;
import org.team498.C2023.commands.robot.Score;
import org.team498.C2023.commands.robot.Spit;
import org.team498.C2023.commands.wrist.ManualWrist;
import org.team498.C2023.commands.wrist.SetWristState;
import org.team498.C2023.subsystems.*;
import org.team498.lib.drivers.Gyro;
import org.team498.lib.drivers.Xbox;

public class Controls {
    public final Xbox driver = new Xbox(OIConstants.DRIVER_CONTROLLER_ID);
    public final Xbox operator = new Xbox(OIConstants.OPERATOR_CONTROLLER_ID);

    private final Drivetrain drivetrain = Drivetrain.getInstance();
    private final Elevator elevator = Elevator.getInstance();
    private final Manipulator manipulator = Manipulator.getInstance();
    private final Wrist wrist = Wrist.getInstance();
    private final Intake intake = Intake.getInstance();
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
        // driver.leftTrigger().onTrue(new SetIntake(Intake.State.INTAKE).alongWith(new SetWristState(Wrist.State.CONEARISER)).alongWith(new SetManipulatorState(Manipulator.State.COLLECT))).onFalse(new SetIntake(Intake.State.IDLE).alongWith(new SetWristState(Wrist.State.SPIT)).alongWith(new SetManipulatorState(Manipulator.State.IDLE)));
        driver.start().onTrue(new PathPlannerFollower(PathLib.singleCubeTaxi));

        // driver.leftBumper().onTrue(new InstantCommand(() -> robotState.setCurrentGameMode(GamePiece.CUBE)));

        driver.rightTrigger().onTrue(new Score());
        // driver.B().onTrue(new CubeEngage());
        driver.leftBumper().onTrue(new Spit());
        driver.leftTrigger().onTrue(new IntakeCommand()).onFalse(new Reset());
        driver.A().onTrue(new InstantCommand(() -> Gyro.getInstance().setYaw(0)));
        driver.X().onTrue(new SetIntakeState(Intake.State.OUTTAKE)).onFalse(new SetIntakeState(Intake.State.IDLE_IN));
    }

    public void configureOperatorCommands() {
        // operator.start().onTrue(new PathPlannerFollower(PathLib.singleCubeTaxi));
        // operator.start().onTrue(new AlignAndExecute(RobotPositions::getLeftScoringPosition));

        operator.Y().onTrue(new InstantCommand(() -> robotState.setNextHeight(Height.TOP)));
        operator.B().onTrue(new InstantCommand(() -> robotState.setNextHeight(Height.MID)));
        operator.A().onTrue(new InstantCommand(() -> robotState.setNextHeight(Height.LOW)));

        operator.X().whileTrue(new FixCube()).onFalse(new LowerElevator().alongWith(new SetIntakeState(Intake.State.IDLE_IN), new StopManipulator()));

        // Left bumper sets the current game piece to a cone, right bumper sets it to a cube
        operator.rightBumper().onTrue(new InstantCommand(() -> robotState.setCurrentGameMode(GamePiece.CONE)));
        operator.leftBumper().onTrue(new InstantCommand(() -> robotState.setCurrentGameMode(GamePiece.CUBE)));

        // operator.start().onTrue(new SetConeARiserState(ConeARiser.State.COLLECT)).onFalse(new SetConeARiserState(ConeARiser.State.IDLE));

        operator.rightTrigger().onTrue(new InstantCommand(() -> robotState.setNextHeight(Height.DOUBLE_SS)).alongWith(new CollectFromSS())).onFalse(new Reset());
        operator.leftTrigger().onTrue(new InstantCommand(() -> robotState.setNextHeight(Height.SINGLE_SS)).alongWith(new CollectFromSS())).onFalse(new Reset());

        operator.start().toggleOnTrue(new ManualElevator(() -> -operator.leftY()));
        operator.back().toggleOnTrue(new ManualWrist(() -> -operator.rightY()));
    }
}