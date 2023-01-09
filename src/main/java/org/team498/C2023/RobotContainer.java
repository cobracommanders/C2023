package org.team498.C2023;

import org.team498.C2023.commands.CalibrateGyro;
import org.team498.C2023.commands.auto.Auto_1;
import org.team498.C2023.commands.drivetrain.AlignWithSubstation;
import org.team498.C2023.commands.drivetrain.AlignWithSubstation.SubstationSide;
import org.team498.C2023.commands.drivetrain.archive.FieldOrientedDrive;
import org.team498.C2023.commands.drivetrain.archive.OffenseDrive;
import org.team498.C2023.commands.drivetrain.archive.RobotOrientedDrive;
import org.team498.C2023.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
	private static RobotContainer mInstance;

	public static RobotContainer getInstance() {
		if (mInstance == null) {
			mInstance = new RobotContainer();
		}
		return mInstance;
	}

	// private final Vision vision = new Vision();
	private final Drivetrain drivetrain = Drivetrain.getInstance();

	private final DriverController driverControls = DriverController.getInstance();

	public RobotContainer() {
		configureDriverBindings();
		drivetrain.setDefaultCommand(new OffenseDrive());
	}


	private void configureDriverBindings() {
		Trigger robotInLoadingZone = new Trigger(() -> drivetrain.isInRegion(null, null)); //TODO add to constants
		driverControls.xButton.and(robotInLoadingZone).onTrue(new AlignWithSubstation(SubstationSide.LEFT));

		// driverControls.aButton.whenPressed(new SetWrist(Wrist.State.OUT));
		// driverControls.bButton.whenPressed(new SetWrist(Wrist.State.IN));
		driverControls.aButton.onTrue(new InstantCommand(() -> drivetrain.IMU.reset()));

		driverControls.getControlSet().toggleOnTrue(new FieldOrientedDrive());
		driverControls.getRobotOriented().toggleOnTrue(new RobotOrientedDrive());
	}

	public Command getAutoCommand() {
		return new Auto_1();
	}

	public Command getRobotInitCommand() {
		return new CalibrateGyro(drivetrain);
	}
}
