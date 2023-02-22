package org.team498.C2023;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.team498.C2023.Constants.OIConstants;
import org.team498.C2023.RobotState.GamePiece;
import org.team498.C2023.RobotState.Height;
import org.team498.C2023.commands.drivetrain.*;
import org.team498.C2023.commands.elevator.ManualElevator;
import org.team498.C2023.commands.elevator.SetElevatorToNextState;
import org.team498.C2023.commands.manipulator.SetManipulatorToNextState;
import org.team498.C2023.commands.manipulator.StopManipulator;
import org.team498.C2023.commands.robot.LowerElevator;
import org.team498.C2023.commands.robot.AlignAndExecute;
import org.team498.C2023.commands.wrist.ManualWrist;
import org.team498.C2023.commands.wrist.SetWristToNextState;
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

        configureDefaultCommands();
        configureDriverCommands();
        configureOperatorCommands();
    }

    private void configureDefaultCommands() {
        drivetrain.setDefaultCommand(
                new DefenseDrive(driver::leftYSquared, driver::leftXSquared, driver::rightX, driver.rightBumper()));
    }

    private void configureDriverCommands() {
        driver.start().onTrue(new PathPlannerFollower(PathLib.SingleCube.Path1));

        // Align for scoring
        driver
                .leftTrigger()
                .and(RobotPositions::inCommunity)
                .and(() -> RobotState.getInstance().inConeMode())
                .onTrue(new AlignAndExecute(RobotPositions::getLeftScoringPosition));
        driver
                .rightTrigger()
                .and(RobotPositions::inCommunity)
                .and(() -> RobotState.getInstance().inConeMode())
                .onTrue(new AlignAndExecute(RobotPositions::getRightScoringPosition));

        driver
                .rightTrigger()
                .or(driver.leftTrigger())
                .and(RobotPositions::inCommunity)
                .and(() -> RobotState.getInstance().inCubeMode())
                .onTrue(new AlignAndExecute(RobotPositions::getCenterScoringPosition));

        // Align for collecting from the double substation
        driver
                .leftTrigger()
                .and(RobotPositions::inLoadingZone)
                .onTrue(new AlignAndExecute(RobotPositions::getLeftSubstationPosition));
        driver
                .rightTrigger()
                .and(RobotPositions::inLoadingZone)
                .onTrue(new AlignAndExecute(RobotPositions::getRightSubstationPosition));
        driver
                .X()
                .and(RobotPositions::inLoadingZone)
                .onTrue(new DriveToPosition(RobotPositions::getSingleSubstationPosition));

        // Intake/Score
        driver.leftBumper().onTrue(new SetManipulatorToNextState()).onFalse(new StopManipulator());
        // Reset gyro
        driver.A().onTrue(new InstantCommand(() -> Gyro.getInstance().setYaw(0)));
        // Move to the next target
        driver.B().onTrue(new SetElevatorToNextState().alongWith(new SetWristToNextState()));
    }

    private void configureOperatorCommands() {
        operator.Y().onTrue(new InstantCommand(() -> robotState.setNextHeight(Height.TOP)));
        operator.B().onTrue(new InstantCommand(() -> robotState.setNextHeight(Height.MID)));
        operator.A().onTrue(new InstantCommand(() -> robotState.setNextHeight(Height.LOW)));
        operator.X().onTrue(new LowerElevator());

        // Left bumper sets the current game piece to a cone, right bumper sets it to a cube
        operator.leftBumper().onTrue(new InstantCommand(() -> robotState.setCurrentGameMode(GamePiece.CONE)));
        operator.rightBumper().onTrue(new InstantCommand(() -> robotState.setCurrentGameMode(GamePiece.CUBE)));

        SmartDashboard.putData(new ManualElevator(() -> -operator.leftY()));
        SmartDashboard.putData(new ManualWrist(() -> -operator.rightY()));
    }
}
