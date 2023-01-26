package org.team498.C2023;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import org.team498.C2023.Constants.OIConstants;
import org.team498.C2023.RobotState.GamePiece;
import org.team498.C2023.commands.drivetrain.*;
import org.team498.C2023.commands.prototype.PrototypeTest;
import org.team498.C2023.commands.robot.AlignAndScore;
import org.team498.C2023.subsystems.*;
import org.team498.C2023.subsystems.Outtake.State;
import org.team498.lib.drivers.Gyro;
import org.team498.lib.drivers.Xbox;

public class RobotContainer {
    public final Xbox driver = new Xbox(OIConstants.DRIVER_CONTROLLER_ID);
    public final Xbox operator = new Xbox(OIConstants.OPERATOR_CONTROLLER_ID);

    private final Drivetrain drivetrain = Drivetrain.getInstance();
    private final Elevator elevator = Elevator.getInstance();
    private final Outtake outtake = Outtake.getInstance();
    private final Intake intake = Intake.getInstance();
    private final Prototype prototype = Prototype.getInstance();
    private final RobotState robotState = RobotState.getInstance();

    public RobotContainer() {
        driver.setDeadzone(0.2);
        driver.setTriggerThreshold(0.2);

        configureDefaultCommands();
        configureDriverCommands();
        configureOperatorCommands();
    }

    private void configureDefaultCommands() {
        drivetrain.setDefaultCommand(new OffenseDrive(driver::leftY, driver::leftX, driver::rightAngle));
        prototype.setDefaultCommand(new PrototypeTest(operator::leftX));
        elevator.setDefaultCommand(elevator.setPosition(Elevator.Position.DRIVING));
    }

    private void configureDriverCommands() {
        driver.back().onTrue(new InstantCommand(() -> {
                if (robotState.getCurrentGamePiece() == GamePiece.CONE) {
                    robotState.setCurrentGamePiece(GamePiece.CUBE);
                } else {
                    robotState.setCurrentGamePiece(GamePiece.CONE);
                }
            }));
    
        driver.start().onTrue(new PathPlannerFollower(PathLib.pathPlannerTrajectory));

        driver.POVMinus90().and(RobotPositions::inCommunity).and(() -> RobotState.getInstance().getCurrentGamePiece() == GamePiece.CONE).onTrue(new AlignAndScore(RobotPositions::getLeftPosition).until(driver::isStickActive));
        driver.POV90().and(RobotPositions::inCommunity).and(() -> RobotState.getInstance().getCurrentGamePiece() == GamePiece.CONE).onTrue(new AlignAndScore(RobotPositions::getRightPosition).until(driver::isStickActive));

        driver.POV90().and(RobotPositions::inCommunity).and(() -> RobotState.getInstance().getCurrentGamePiece() == GamePiece.CUBE).onTrue(new AlignAndScore(RobotPositions::getCenterPosition).until(driver::isStickActive));
        driver.POVMinus90().and(RobotPositions::inCommunity).and(() -> RobotState.getInstance().getCurrentGamePiece() == GamePiece.CUBE).onTrue(new AlignAndScore(RobotPositions::getCenterPosition).until(driver::isStickActive));

        driver.rightTrigger().onTrue(intake.setIntakeMode(Intake.State.INTAKE)).onFalse(intake.setIntakeMode(Intake.State.IDLE));

        driver.rightTrigger().onTrue(intake.setIntakeMode(Intake.State.OUTTAKE)).onFalse(intake.setIntakeMode(Intake.State.IDLE));

        driver.rightBumper().and(RobotPositions::inCommunity).onTrue(outtake.setOuttake(State.SHOOT_CONE)).onFalse(outtake.setOuttake(State.IDLE));
        driver.leftBumper().and(RobotPositions::inCommunity).onTrue(outtake.setOuttake(State.SHOOT_CUBE)).onFalse(outtake.setOuttake(State.IDLE));

        driver.A().onTrue(new InstantCommand(() -> Gyro.getInstance().setYaw(0)));
        driver.B().toggleOnTrue(new TargetDrive(driver::leftY, driver::leftX, new Pose2d(6, 4, new Rotation2d())));
        driver.X().toggleOnTrue(new DefenseDrive(driver::leftY, driver::leftX, driver::rightX));
        driver.Y().toggleOnTrue(new CameraDrive(driver::leftY, driver::leftX, driver::rightX));
    }

    private void configureOperatorCommands() {
        operator.Y().onTrue(elevator.setNextHeight(Elevator.Position.HIGH)).and(RobotPositions::inCommunity).onTrue(elevator.setPosition(Elevator.Position.HIGH));
        operator.X().onTrue(elevator.setNextHeight(Elevator.Position.MID)).and(RobotPositions::inCommunity).onTrue(elevator.setPosition(Elevator.Position.MID));
        operator.A().onTrue(elevator.setNextHeight(Elevator.Position.LOW)).and(RobotPositions::inCommunity).onTrue(elevator.setPosition(Elevator.Position.LOW));
        operator.B().onTrue(elevator.setNextHeight(Elevator.Position.FLOOR)).and(RobotPositions::inCommunity).onTrue(elevator.setPosition(Elevator.Position.FLOOR));

        operator.start().onTrue(new InstantCommand(() -> robotState.setCurrentGamePiece(GamePiece.CONE)));
        operator.back().onTrue(new InstantCommand(() -> robotState.setCurrentGamePiece(GamePiece.CUBE)));
    }
}