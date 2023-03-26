package org.team498.C2023.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team498.C2023.Constants;
import org.team498.C2023.Ports;
import org.team498.C2023.Robot;
import org.team498.lib.drivers.gyro.GyroIO;
import org.team498.lib.drivers.gyro.GyroIOInputsAutoLogged;
import org.team498.lib.drivers.gyro.GyroIOPigeon2;
import org.team498.lib.drivers.swervemodule.ModuleIO;
import org.team498.lib.drivers.swervemodule.ModuleIOFalcon500;
import org.team498.lib.drivers.swervemodule.ModuleIOInputsAutoLogged;
import org.team498.lib.drivers.swervemodule.ModuleIOSim;
import org.team498.lib.logger.LoggerUtil;
import org.team498.lib.wpilib.ChassisSpeeds;

import static org.team498.C2023.Constants.DrivetrainConstants.*;

public class Drivetrain extends SubsystemBase {
    private final ModuleIO[] modules;
    private final ModuleIOInputsAutoLogged[] moduleInputs = new ModuleIOInputsAutoLogged[] {new ModuleIOInputsAutoLogged(), new ModuleIOInputsAutoLogged(), new ModuleIOInputsAutoLogged(), new ModuleIOInputsAutoLogged()};
    private final ProfiledPIDController angleController = new ProfiledPIDController(AngleConstants.P, AngleConstants.I, AngleConstants.D, AngleConstants.CONTROLLER_CONSTRAINTS);
    private final PIDController xController = new PIDController(PoseConstants.P, PoseConstants.I, PoseConstants.D);
    private final PIDController yController = new PIDController(PoseConstants.P, PoseConstants.I, PoseConstants.D);
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private SwerveModuleState[] stateSetpoints;

    private final GyroIO gyro;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private Drivetrain() {
        modules = switch (Constants.mode) {
            case REAL, REPLAY, PRACTICE ->
                new ModuleIO[] {
                    new ModuleIOFalcon500(ModuleIOFalcon500.Module.FL),
                    new ModuleIOFalcon500(ModuleIOFalcon500.Module.FR),
                    new ModuleIOFalcon500(ModuleIOFalcon500.Module.BL),
                    new ModuleIOFalcon500(ModuleIOFalcon500.Module.BR),
                    };
            case SIM -> new ModuleIO[] {
                    new ModuleIOSim("FL"),
                    new ModuleIOSim("FR"),
                    new ModuleIOSim("BL"),
                    new ModuleIOSim("BR")
            };
        };

        gyro = new GyroIOPigeon2(Ports.Drivetrain.GYRO);

        angleController.enableContinuousInput(-180, 180);
        angleController.setTolerance(0);
        //TODO Reset angleController
        xController.setTolerance(0);
        yController.setTolerance(0);
        xLimiter.reset(0);
        yLimiter.reset(0);

        kinematics = new SwerveDriveKinematics(getModuleTranslations());
        odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(getYaw()), getModulePositions());

        stateSetpoints = getModuleStates();

        for (ModuleIO m : modules) m.setBrakeMode(true);
    }

    @Override
    public void periodic() {
        for (int i = 0; i < modules.length; i++) {
            modules[i].updateInputs(moduleInputs[i]);
            Logger.getInstance().processInputs("Drive/" + modules[i].getName() + "_Module", moduleInputs[i]);

            // modules[i].setBrakeMode(RobotState.isEnabled());

            if (RobotState.isDisabled()) {
                modules[i].updateIntegratedEncoder();
            }
        }
        gyro.updateInputs(gyroInputs);
        Logger.getInstance().processInputs("Gyro", gyroInputs);
        odometry.update(Rotation2d.fromDegrees(getYaw()), getModulePositions());
        //TODO See if SwerveDrivePoseEstimator could replace resetting the odometry so we could track both
        Logger.getInstance().recordOutput("Odometry", getPose());
        
        LoggerUtil.recordOutput("Drive/RealStates", getModuleStates());

        var targetStates = new SwerveModuleState[4];
        for (int i = 0; i < modules.length; i++) {
            targetStates[i] = new SwerveModuleState(moduleInputs[i].targetSpeedMetersPerSecond, Rotation2d.fromDegrees(moduleInputs[i].targetAngle));
        }
        LoggerUtil.recordOutput("Drive/TargetStates", targetStates);

        if (!gyroInputs.connected) {
            gyro.setYaw(gyroInputs.yaw + Math.toDegrees(getCurrentSpeeds().omegaRadiansPerSecond) * Robot.DEFAULT_PERIOD);
        }

        Robot.field.setRobotPose(getPose());

    }

    public void drive(double vx, double vy, double degreesPerSecond, boolean fieldOriented) {
        ChassisSpeeds speeds = fieldOriented
                               ? ChassisSpeeds.fromFieldRelativeSpeeds(-vx, -vy, Math.toRadians(degreesPerSecond), getYaw())
                               : new ChassisSpeeds(-vx, -vy, Math.toRadians(degreesPerSecond));

        speeds.vxMetersPerSecond = xLimiter.calculate(speeds.vxMetersPerSecond);
        speeds.vyMetersPerSecond = yLimiter.calculate(speeds.vyMetersPerSecond);

        var states = kinematics.toSwerveModuleStates(speeds);

        stateSetpoints = states;

        setModuleStates(stateSetpoints);
    }

    public void setModuleStates(SwerveModuleState[] states) {
        for (int i = 0; i < modules.length; i++) modules[i].setState(states[i]);
    }

    public SwerveModulePosition[] getModulePositions() {
        var positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) positions[i] = new SwerveModulePosition(moduleInputs[i].positionMeters, Rotation2d.fromDegrees(moduleInputs[i].angle));
        return positions;
    }

    public SwerveModuleState[] getModuleStates() {
        var states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) states[i] = new SwerveModuleState(moduleInputs[i].speedMetersPerSecond, Rotation2d.fromDegrees(moduleInputs[i].angle));
        return states;
    }

    public void setPositionGoal(Pose2d pose) {xController.setSetpoint(pose.getX()); yController.setSetpoint(pose.getY()); setAngleGoal(pose.getRotation().getDegrees()); Robot.field.getObject("targetPose").setPose(pose); Logger.getInstance().recordOutput("TargetPose", pose);}
    public ChassisSpeeds calculatePositionSpeed() {return new ChassisSpeeds(xController.calculate(getPose().getX()) * Robot.coordinateFlip, yController.calculate(getPose().getY()) * Robot.coordinateFlip, calculateAngleSpeed());}
    public boolean atPositionGoal() {return (Math.abs(xController.getPositionError()) < PoseConstants.EPSILON) && (Math.abs(yController.getPositionError()) < PoseConstants.EPSILON) && atAngleGoal();}

    public void setXGoal(double pose) {xController.setSetpoint(pose);}
    public double calculateXSpeed() {return xController.calculate(getPose().getX());}
    public boolean atXGoal() {return Math.abs(xController.getPositionError()) < PoseConstants.EPSILON;}
    
    public void setYGoal(double pose) {yController.setSetpoint(pose);}
    public double calculateYSpeed() {return yController.calculate(getPose().getY());}
    public boolean atYGoal() {return Math.abs(yController.getPositionError()) < PoseConstants.EPSILON;}

    public void setAngleGoal(double angle) {angleController.setGoal(angle);}
    public double calculateAngleSpeed() {return angleController.calculate(getYaw());}
    public boolean atAngleGoal() {return Math.abs(angleController.getPositionError()) < AngleConstants.EPSILON;}

    public Pose2d getPose() {return odometry.getPoseMeters();}
    public void setPose(Pose2d pose) {odometry.resetPosition(Rotation2d.fromDegrees(0), getModulePositions(), pose);}
    public double getYaw() {return gyroInputs.yaw;}
    public void setYaw(double angle) {gyro.setYaw(angle);}
    /** Return a double array with a value for yaw pitch and roll in that order */
    public double[] getGyro() {return new double[] {gyroInputs.yaw, gyroInputs.pitch, gyroInputs.roll};}

    public void stop() {drive(0, 0, 0, false);}
    public void X() {setModuleStates(new SwerveModuleState[] {new SwerveModuleState(0, Rotation2d.fromDegrees(45)), new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), new SwerveModuleState(0, Rotation2d.fromDegrees(45))});}

    public ChassisSpeeds getCurrentSpeeds() {return ChassisSpeeds.fromWPIChassisSpeeds(kinematics.toChassisSpeeds(getModuleStates()));}

    private Translation2d[] getModuleTranslations() {
        double moduleDistance = Units.inchesToMeters(SWERVE_MODULE_DISTANCE_FROM_CENTER);
        Translation2d FL_ModulePosition = new Translation2d(moduleDistance, moduleDistance);
        Translation2d FR_ModulePosition = new Translation2d(moduleDistance, -moduleDistance);
        Translation2d BL_ModulePosition = new Translation2d(-moduleDistance, moduleDistance);
        Translation2d BR_ModulePosition = new Translation2d(-moduleDistance, -moduleDistance);
        return new Translation2d[] {FL_ModulePosition, FR_ModulePosition, BL_ModulePosition, BR_ModulePosition};
    }

    private static Drivetrain instance;

    public static Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain();
        }
        return instance;
    }
}
