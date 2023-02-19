package org.team498.C2023;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.team498.C2023.Constants.OIConstants;
import org.team498.C2023.RobotState.GamePiece;
import org.team498.C2023.commands.drivetrain.*;
import org.team498.C2023.commands.elevator.ManualElevator;
import org.team498.C2023.commands.elevator.SetElevatorState;
import org.team498.C2023.commands.manipulator.CollectGamePiece;
import org.team498.C2023.commands.manipulator.HoldCone;
import org.team498.C2023.commands.manipulator.ScoreGamePiece;
import org.team498.C2023.commands.manipulator.StopManipulator;
import org.team498.C2023.commands.robot.AlignAndScore;
import org.team498.C2023.commands.robot.RetrieveFromDoubleSS;
import org.team498.C2023.commands.robot.Score;
import org.team498.C2023.commands.wrist.SetWristState;
import org.team498.C2023.subsystems.*;
import org.team498.lib.drivers.Gyro;
import org.team498.lib.drivers.Xbox;

public class RobotContainer {
    public final Xbox driver = new Xbox(OIConstants.DRIVER_CONTROLLER_ID);
    public final Xbox operator = new Xbox(OIConstants.OPERATOR_CONTROLLER_ID);

    private final Drivetrain drivetrain = Drivetrain.getInstance();
    private final Elevator elevator = Elevator.getInstance();
    private final Manipulator manipulator = Manipulator.getInstance();
    private final Wrist wrist = Wrist.getInstance();
    private final Intake intake = Intake.getInstance();
    private final ConeARiser coneARiser = ConeARiser.getInstance();
    private final RobotState robotState = RobotState.getInstance();

    public RobotContainer() {
        driver.setDeadzone(0.2);
        driver.setTriggerThreshold(0.2);
        operator.setDeadzone(0.2);
        operator.setTriggerThreshold(0.2);

        configureDefaultCommands();
        configureDriverCommands();
        configureOperatorCommands();
    }

    private void configureDefaultCommands() {
        drivetrain.setDefaultCommand(new DefenseDrive(driver::leftYSquared, driver::leftXSquared, driver::rightX, driver.rightBumper()));
        // manipulator.setDefaultCommand(new HoldCone());
        // elevator.setDefaultCommand(new ManualElevator(operator::leftY));
    }

    private void configureDriverCommands() {
        driver.start().onTrue(new PathPlannerFollower(PathLib.SingleCube.Path1));

        driver
                .POVMinus90()
                .and(RobotPositions::inCommunity)
                .and(() -> RobotState.getInstance().hasCone())
                .onTrue(new AlignAndScore(RobotPositions::getLeftPosition).until(driver::isStickActive));
        driver
                .POV90()
                .and(RobotPositions::inCommunity)
                .and(() -> RobotState.getInstance().hasCone())
                .onTrue(new AlignAndScore(RobotPositions::getRightPosition).until(driver::isStickActive));

        driver
                .POV90()
                .or(driver.POVMinus90())
                .and(RobotPositions::inCommunity)
                .and(() -> RobotState.getInstance().hasCube())
                .onTrue(new AlignAndScore(RobotPositions::getCenterPosition).until(driver::isStickActive));

        // Raise elevator and score
        driver.leftTrigger().onTrue(new ScoreGamePiece()).onFalse(new StopManipulator());

        // Intake a cone
        driver.leftBumper().onTrue(new CollectGamePiece()).onFalse(new StopManipulator());

        driver.A().onTrue(new InstantCommand(() -> Gyro.getInstance().setYaw(0)));
        driver.B().toggleOnTrue(new TargetDrive(driver::leftYSquared, driver::leftXSquared, new Pose2d(6, 4, new Rotation2d())));
        driver.X().toggleOnTrue(new OffenseDrive(driver::leftYSquared, driver::leftXSquared, driver::rightAngle, driver.rightBumper()));
        driver.Y().toggleOnTrue(new CameraDrive(driver::leftYSquared, driver::leftXSquared, driver::rightXSquared));
    }

    private void configureOperatorCommands() {
        operator.Y().onTrue(new SetElevatorState(Elevator.State.HIGH));
        operator.X().onTrue(new SetElevatorState(Elevator.State.MID));
        operator.A().onTrue(new SetElevatorState(Elevator.State.LOW));

        // operator.Y().onTrue(new SetWristState(Wrist.State.SCORE_CUBE));
        // operator.X().onTrue(new SetWristState(Wrist.State.SCORE_CONE));
        // operator.Y().onTrue(new SetWristState(Wrist.State.UP));
        // operator.B().onTrue(new SetWristState(Wrist.State.IDLE));

        // Reset the drivetrain pose to the field corner for driver practice
        // operator.start().onTrue(new InstantCommand(() -> drivetrain.setPose(new Pose2d(Units.inchesToMeters(70.5),
        //                                                                                Units.inchesToMeters(16.5),
        //                                                                                new Rotation2d()
        // ))));

        operator.start().onTrue(new RetrieveFromDoubleSS());

        // Left bumper sets the current game piece to a cone, right bumper sets it to a cube
        operator.leftBumper().onTrue(new InstantCommand(() -> robotState.setCurrentGamePiece(GamePiece.CONE)));
        operator.rightBumper().onTrue(new InstantCommand(() -> robotState.setCurrentGamePiece(GamePiece.CUBE)));
    }
}

