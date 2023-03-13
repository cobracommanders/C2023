package org.team498.lib.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderSimCollection;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team498.lib.util.Falcon500Conversions;

import static org.team498.C2023.Constants.DrivetrainConstants.*;

import org.team498.C2023.Robot;

public class SwerveModule extends SubsystemBase {
    private final WPI_TalonFX driveMotor;
    private final WPI_TalonFX steerMotor;
    private final WPI_CANCoder encoder;
    private final double angleOffset;
    private final String name;

    private double lastAngle;

    private final TalonFXSimCollection simDriveMotor;
    private final TalonFXSimCollection simSteerMotor;
    private final CANCoderSimCollection simEncoder;
    private final FlywheelSim driveSim;
    private final FlywheelSim steerSim;

    public SwerveModule(String name, WPI_TalonFX driveMotor, WPI_TalonFX steerMotor, WPI_CANCoder CANCoder, double angleOffset) {
        this.name = name;
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;
        this.encoder = CANCoder;
        configDriveMotor(driveMotor);
        configSteerMotor(steerMotor);
        configCANCoder(CANCoder);

        this.angleOffset = angleOffset;
        this.lastAngle = getState().angle.getDegrees();
        matchEncoders();


        if (Robot.isSimulation()) {
            simDriveMotor = driveMotor.getSimCollection();
            simSteerMotor = steerMotor.getSimCollection();
            simEncoder = encoder.getSimCollection();
            steerSim = new FlywheelSim(DCMotor.getFalcon500(1), MK4I_STEER_REDUCTION_L2, 0.004096955);
            driveSim = new FlywheelSim(DCMotor.getFalcon500(1), MK4I_DRIVE_REDUCTION_L2, 0.025);

            encoder.setPosition(angleOffset);
        } else {
            simDriveMotor = null;
            simSteerMotor = null;
            simEncoder = null;
            steerSim = null;
            driveSim = null;
        }

    }


    /** Matches the integrated encoder to the reading from the CANCoder */
    public void matchEncoders() {
        steerMotor.setSelectedSensorPosition(Falcon500Conversions.degreesToFalcon(encoder.getAbsolutePosition() - angleOffset,
                                                                                  MK4I_STEER_REDUCTION_L2
        ));
    }

    /** Return the position of the wheel based on the integrated motor encoder */
    public double getSteerEncoder() {
        return Falcon500Conversions.falconToDegrees(steerMotor.getSelectedSensorPosition(), MK4I_STEER_REDUCTION_L2) + angleOffset;
    }

    public SwerveModuleState currentTarget = new SwerveModuleState();
    public boolean forcedAngle = false;

    /** Sets the motors of the swerve module to a provided state */
    public void setState(SwerveModuleState state, boolean force) {
        this.currentTarget = optimize(state, getSteerEncoder());
        this.forcedAngle = force;
    }

    public SwerveModuleState getTargetState() {
        return currentTarget;
    }

    @Override
    public void periodic() {
        double velocity = Falcon500Conversions.MPSToFalcon(currentTarget.speedMetersPerSecond, Units.inchesToMeters(DRIVE_WHEEL_DIAMETER), MK4I_DRIVE_REDUCTION_L2);
        driveMotor.set(ControlMode.Velocity, velocity);

        double angle = (Math.abs(velocity) <= MAX_VELOCITY_METERS_PER_SECOND * 0.01) && !forcedAngle
                       ? lastAngle
                       : currentTarget.angle.getDegrees();
        steerMotor.set(ControlMode.Position, Falcon500Conversions.degreesToFalcon(angle - angleOffset, MK4I_STEER_REDUCTION_L2));

        lastAngle = getState().angle.getDegrees();

        SmartDashboard.putNumber(name, getSteerEncoder());
    }

    @Override
    public void simulationPeriodic() {
        encoder.setPosition(Falcon500Conversions.falconToDegrees(steerMotor.getSelectedSensorPosition(), MK4I_STEER_REDUCTION_L2));

        steerSim.setInputVoltage(steerMotor.getMotorOutputVoltage());
        steerSim.update(Robot.kDefaultPeriod);
        steerMotor.setSelectedSensorPosition(steerMotor.getSelectedSensorPosition() + ((steerSim.getAngularVelocityRPM() / MK4I_STEER_REDUCTION_L2) * 2048 * Robot.kDefaultPeriod));

        driveSim.setInputVoltage(driveMotor.getMotorOutputVoltage());
        driveSim.update(Robot.kDefaultPeriod);
        driveMotor.setSelectedSensorPosition(driveMotor.getSelectedSensorPosition() + ((driveSim.getAngularVelocityRPM() / MK4I_DRIVE_REDUCTION_L2) * 2048 * Robot.kDefaultPeriod));
	}

    /** Get the velocity of the wheel in meters per second */
    private double getVelocityMPS() {
        // Convert the value returned by the sensor (rotations per 100ms) to rotations per second
        return Falcon500Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(),
                                                Units.inchesToMeters(DRIVE_WHEEL_CIRCUMFERENCE),
                                                MK4I_DRIVE_REDUCTION_L2
        );
    }

    /** Get the current state of the swerve module as a {@link SwerveModuleState} */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                // Velocity of the wheel
                getVelocityMPS(),
                // The value of the steering encoder
                Rotation2d.fromDegrees(getSteerEncoder())
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), Rotation2d.fromDegrees(getSteerEncoder()));
    }

    private double getPositionMeters() {
        return (Falcon500Conversions.falconToDegrees(driveMotor.getSelectedSensorPosition(), MK4I_DRIVE_REDUCTION_L2) / 360) * Units.inchesToMeters(
                DRIVE_WHEEL_CIRCUMFERENCE);
    }

    public Pose2d getPose(Pose2d swerve, Translation2d relativePosition) {
		// Takes the current position of the robot and adds to it the relative position
		// of the swerve module
		return swerve.plus(
				new Transform2d(
						relativePosition, 
						// Also sets the rotation of the module to the value of the steering encoder
						Rotation2d.fromDegrees(getSteerEncoder())));
	}

    // Custom optimize method by team 364
    public static SwerveModuleState optimize(SwerveModuleState desiredState, double currentAngle) {
        double targetAngle = placeInAppropriate0To360Scope(currentAngle, desiredState.angle.getDegrees());

        double targetSpeed = desiredState.speedMetersPerSecond;

        double delta = targetAngle - currentAngle;

        if (Math.abs(delta) > 90) {
            targetSpeed = -targetSpeed;
            if (delta > 90) {targetAngle -= 180;} else {targetAngle += 180;}
        }
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }

    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
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

    private void configDriveMotor(TalonFX motor) {
        motor.configFactoryDefault();
        TalonFXConfiguration currentLimitConfig = new TalonFXConfiguration();

        currentLimitConfig.supplyCurrLimit.currentLimit = 35;
        currentLimitConfig.supplyCurrLimit.enable = true;

        motor.configAllSettings(currentLimitConfig);

        motor.setNeutralMode(NeutralMode.Brake);

        motor.configOpenloopRamp(1);
        motor.setSelectedSensorPosition(0);

        motor.config_kP(0, 0.025);
        motor.config_kI(0, 0.0);
        motor.config_kD(0, 0.5);

        motor.setInverted(false);
    }

    private void configSteerMotor(TalonFX motor) {
        motor.configFactoryDefault();
        TalonFXConfiguration currentLimitConfig = new TalonFXConfiguration();

        currentLimitConfig.supplyCurrLimit.currentLimit = 20;
        currentLimitConfig.supplyCurrLimit.enable = true;

        motor.configAllSettings(currentLimitConfig);

        motor.setNeutralMode(NeutralMode.Brake);
        motor.configOpenloopRamp(1);
        motor.setSelectedSensorPosition(0);

        motor.setSensorPhase(true);

        motor.config_kP(0, 0.2);
        motor.config_kI(0, 0.0);
        motor.config_kD(0, 0.1);

        motor.setInverted(true);
    }

    private void configCANCoder(CANCoder CANCoder) {
        // Sets the encoder to boot to the absolute position instead of 0
        CANCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

        // Set the encoder to return values from 0 to 360 instead of -180 to +180
        CANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        // if (name.equals("FL")) {
            // CANCoder.configMagnetOffset(angleOffset + 180);
        // } else {
            // CANCoder.configMagnetOffset(angleOffset);
        // }

    }

}