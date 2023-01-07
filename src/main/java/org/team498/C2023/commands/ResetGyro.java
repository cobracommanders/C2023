package org.team498.C2023.commands;

import org.team498.C2023.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;

// Sets the gyro sensor angle to 0
public class ResetGyro extends InstantCommand {
	private Drivetrain drivetrain;

	public ResetGyro(Drivetrain drivetrain) {
		this.drivetrain = drivetrain;
	}

	@Override
	public void initialize() {
		drivetrain.zeroGyro();
	}
}
