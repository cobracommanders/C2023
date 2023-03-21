package org.team498.lib.drivers.swervemodule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import org.team498.C2023.Robot;
import org.team498.lib.util.Falcon500Conversions;

import static org.team498.C2023.Constants.DrivetrainConstants.*;

public class ModuleIOSim implements ModuleIO {
    private final FlywheelSim driveSim;
    private final FlywheelSim steerSim;

    private final PIDController drivePID = new PIDController(1.5, 0, 0.003);
    private final PIDController steerPID = new PIDController(0.02, 0, 0.1);
    private final String name;

    private double angle = 0.0;
    private SwerveModuleState currentTarget = new SwerveModuleState();

    public ModuleIOSim(String name) {
        steerSim = new FlywheelSim(DCMotor.getFalcon500(1), MK4I_STEER_REDUCTION_L2, 0.004096955);
        driveSim = new FlywheelSim(DCMotor.getFalcon500(1), MK4I_DRIVE_REDUCTION_L2, 0.025);

        this.name = name;
    }

    @Override
    public void setState(SwerveModuleState state) {
        currentTarget = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(angle));

        drivePID.setSetpoint(currentTarget.speedMetersPerSecond);
        steerPID.setSetpoint(currentTarget.angle.getDegrees());
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        steerPID.calculate(inputs.angle);
        drivePID.calculate(inputs.speedMetersPerSecond);
        
        double driveVoltage = drivePID.calculate(inputs.speedMetersPerSecond);
        double steerVoltage = steerPID.calculate(inputs.angle);
        driveSim.setInputVoltage(driveVoltage * 12);
        steerSim.setInputVoltage(steerVoltage * 12);

        driveSim.update(Robot.DEFAULT_PERIOD);
        steerSim.update(Robot.DEFAULT_PERIOD);
        angle = inputs.angle;

        inputs.positionMeters = inputs.positionMeters + (Units.inchesToMeters(DRIVE_WHEEL_CIRCUMFERENCE) * ((Math.toDegrees(driveSim.getAngularVelocityRadPerSec()) * Robot.DEFAULT_PERIOD) / 360));
        inputs.speedMetersPerSecond = Units.inchesToMeters(DRIVE_WHEEL_CIRCUMFERENCE) * (Math.toDegrees(driveSim.getAngularVelocityRadPerSec()) / 360);
        inputs.angle = inputs.angle + Math.toDegrees(steerSim.getAngularVelocityRadPerSec()) * Robot.DEFAULT_PERIOD;

        inputs.targetSpeedMetersPerSecond = currentTarget.speedMetersPerSecond;
        inputs.targetAngle = currentTarget.angle.getDegrees();

        inputs.driveAppliedVolts = driveVoltage;
        inputs.driveCurrentAmps = driveSim.getCurrentDrawAmps();
        inputs.driveTemp = 0.0;
        inputs.driveRawEncoder = 0.0;

        inputs.steerAppliedVolts = steerVoltage;
        inputs.steerCurrentAmps = steerSim.getCurrentDrawAmps();
        inputs.steerTemp = 0.0;
        inputs.steerRawEncoder = 0.0;
    }

    // Custom optimize method by team 364
    private SwerveModuleState optimize(SwerveModuleState desiredState, double currentAngle) {
        double targetAngle = placeInAppropriate0To360Scope(currentAngle, desiredState.angle.getDegrees());

        double targetSpeed = desiredState.speedMetersPerSecond;

        double delta = targetAngle - currentAngle;

        if (Math.abs(delta) > 90) {
            targetSpeed = -targetSpeed;
            if (delta > 90) {targetAngle -= 180;} else {targetAngle += 180;}
        }
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }

    private double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
    }

    @Override
    public String getName() {
        return name;
    }
}
