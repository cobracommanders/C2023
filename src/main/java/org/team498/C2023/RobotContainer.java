package org.team498.C2023;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.team498.C2023.Constants.OIConstants;
import org.team498.C2023.RobotState.GamePiece;
import org.team498.C2023.commands.drivetrain.*;
import org.team498.C2023.commands.elevator.ManualElevator;
import org.team498.C2023.commands.elevator.MoveToDoubleSSPosition;
import org.team498.C2023.commands.elevator.MoveToScoringPosition;
import org.team498.C2023.commands.manipulator.CollectGamePiece;
import org.team498.C2023.commands.manipulator.ScoreGamePiece;
import org.team498.C2023.commands.manipulator.StopManipulator;
import org.team498.C2023.commands.robot.AlignAndScore;
import org.team498.C2023.commands.robot.CollectFromDoubleSS;
import org.team498.C2023.commands.robot.LowerElevator;
import org.team498.C2023.commands.wrist.ManualWrist;
import org.team498.C2023.commands.wrist.RotateToDoubleSSPosition;
import org.team498.C2023.commands.wrist.RotateToScoringPosition;
import org.team498.C2023.subsystems.*;
import org.team498.lib.drivers.Gyro;
import org.team498.lib.drivers.Xbox;

import static org.team498.C2023.RobotState.ScoringHeight.TOP;
import static org.team498.C2023.RobotState.ScoringHeight.MID;
import static org.team498.C2023.RobotState.ScoringHeight.LOW;

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
        drivetrain.setDefaultCommand(
                new DefenseDrive(driver::leftYSquared, driver::leftXSquared, driver::rightX, driver.rightBumper()));
        // manipulator.setDefaultCommand(new HoldCone());
        // elevator.setDefaultCommand(new ManualElevator(operator::leftY));
    }

    private void configureDriverCommands() {
        driver.start().onTrue(new PathPlannerFollower(PathLib.SingleCube.Path1));

        // Align for scoring
        driver
                .leftTrigger()
                // .and(RobotPositions::inCommunity)
                .and(() -> RobotState.getInstance().inConeMode())
                .onTrue(new AlignAndScore(RobotPositions::getLeftScoringPosition));
        driver
                .rightTrigger()
                // .and(RobotPositions::inCommunity)
                .and(() -> RobotState.getInstance().inConeMode())
                .onTrue(new AlignAndScore(RobotPositions::getRightScoringPosition));

        driver
                .rightTrigger()
                .or(driver.leftTrigger())
                // .and(RobotPositions::inCommunity)
                .and(() -> RobotState.getInstance().inCubeMode())
                .onTrue(new AlignAndScore(RobotPositions::getCenterScoringPosition));

        // Align for collecting from the double substation
        driver
                .leftTrigger()
                .and(RobotPositions::inLoadingZone)
                .onTrue(new CollectFromDoubleSS(RobotPositions::getLeftSubstationPosition));
        driver
                .rightTrigger()
                .and(RobotPositions::inLoadingZone)
                .onTrue(new CollectFromDoubleSS(RobotPositions::getRightSubstationPosition));
        // driver
                // .X()
                // .and(RobotPositions::inLoadingZone)
                // .onTrue(new DriveToPosition(RobotPositions::getSingleSubstationPosition));

        // Intake a cone
        driver.leftBumper().onTrue(new CollectGamePiece()).onFalse(new StopManipulator());

        driver.A().onTrue(new InstantCommand(() -> Gyro.getInstance().setYaw(0)));
        driver.B().onTrue(new MoveToScoringPosition().alongWith(new RotateToScoringPosition()));
        driver.Y().onTrue(new ScoreGamePiece()).onFalse(new StopManipulator());
        driver.X().onTrue(new MoveToDoubleSSPosition().alongWith(new RotateToDoubleSSPosition()));
    }

    private void configureOperatorCommands() {
        operator.Y().onTrue(new InstantCommand(() -> robotState.setNextScoringHeight(TOP)));
        operator.B().onTrue(new InstantCommand(() -> robotState.setNextScoringHeight(MID)));
        operator.A().onTrue(new InstantCommand(() -> robotState.setNextScoringHeight(LOW)));
        operator.X().onTrue(new LowerElevator());

        // Left bumper sets the current game piece to a cone, right bumper sets it to a cube
        operator.leftBumper().onTrue(new InstantCommand(() -> robotState.setCurrentGameMode(GamePiece.CONE)));
        operator.rightBumper().onTrue(new InstantCommand(() -> robotState.setCurrentGameMode(GamePiece.CUBE)));

        SmartDashboard.putData(new ManualElevator(() -> {
                return -operator.leftY();
        }));

        SmartDashboard.putData(new ManualWrist(() -> {
                return -operator.rightY();
        }));
    }
}
