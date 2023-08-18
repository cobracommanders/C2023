package org.team498.C2023;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.team498.C2023.Constants.OIConstants;
import org.team498.C2023.subsystems.IntakeWrist;
import org.team498.C2023.subsystems.Elevator;
import org.team498.C2023.commands.drivetrain.DefenseDrive;
import org.team498.C2023.commands.elevator.ManualElevator;
import org.team498.C2023.commands.elevator.SetElevatorState;
import org.team498.C2023.commands.intakeWrist.ManualIntakeWrist;
import org.team498.C2023.commands.intakeWrist.SetIntakeWristState;
import org.team498.C2023.subsystems.Drivetrain;
import org.team498.lib.drivers.Gyro;
import org.team498.lib.drivers.Xbox;

public class Controls {
    public final Xbox driver = new Xbox(OIConstants.DRIVER_CONTROLLER_ID);
    public final Xbox operator = new Xbox(OIConstants.OPERATOR_CONTROLLER_ID);

    public Controls() {
        driver.setDeadzone(0.15);
        driver.setTriggerThreshold(0.2);
        operator.setDeadzone(0.2);
        operator.setTriggerThreshold(0.2);
    }

    public void configureDefaultCommands() {
        Elevator.getInstance().setDefaultCommand(new ManualElevator(operator::leftYSquared));
        Drivetrain.getInstance().setDefaultCommand(new DefenseDrive(driver::leftYSquared, driver::leftXSquared, driver::rightX, driver.rightBumper()));
        IntakeWrist.getInstance().setDefaultCommand(new ManualIntakeWrist(operator::rightYSquared));
    }

    public void configureDriverCommands() {
        driver.A().onTrue(new InstantCommand(() -> Gyro.getInstance().setYaw(0)));
    }

    /**
     * 
     */
    public void configureOperatorCommands() {
        operator.A().onTrue(new SetElevatorState(State.Elevator.MID_CONE));
        operator.B().onTrue(new SetElevatorState(State.Elevator.IDLE));
        operator.X().onTrue(new SetIntakeWristState(State.IntakeWrist.INTAKE));
        operator.Y().onTrue(new SetIntakeWristState(State.IntakeWrist.IDLE_IN));
    }
}