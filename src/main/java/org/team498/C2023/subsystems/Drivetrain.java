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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team498.C2023.Constants;
import org.team498.lib.drivers.swervemodule.ModuleIO;
import org.team498.lib.drivers.swervemodule.ModuleIOFalcon500;
import org.team498.lib.drivers.swervemodule.ModuleIOInputsAutoLogged;
import org.team498.lib.wpilib.ChassisSpeeds;

import static org.team498.C2023.Constants.DrivetrainConstants.*;

public class Drivetrain extends SubsystemBase {
    private final ModuleIO[] modules;
    private final ModuleIOInputsAutoLogged[] inputs = new ModuleIOInputsAutoLogged[] {new ModuleIOInputsAutoLogged(), new ModuleIOInputsAutoLogged(), new ModuleIOInputsAutoLogged(), new ModuleIOInputsAutoLogged()};
    private final ProfiledPIDController angleController = new ProfiledPIDController(AngleConstants.P, AngleConstants.I, AngleConstants.D, AngleConstants.CONTROLLER_CONSTRAINTS);
    private final PIDController xController = new PIDController(PoseConstants.P, PoseConstants.I, PoseConstants.D);
    private final PIDController yController = new PIDController(PoseConstants.P, PoseConstants.I, PoseConstants.D);
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private SwerveModuleState[] lastStates;

    private Drivetrain() {
        modules = switch (Constants.mode) {
            case REAL, REPLAY, PRACTICE -> new ModuleIO[] {
                    new ModuleIOFalcon500(ModuleIOFalcon500.Module.FL),
                    new ModuleIOFalcon500(ModuleIOFalcon500.Module.FR),
                    new ModuleIOFalcon500(ModuleIOFalcon500.Module.BL),
                    new ModuleIOFalcon500(ModuleIOFalcon500.Module.BR),
                    };
            case SIM -> new ModuleIO[] {new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}};
        };

        angleController.enableContinuousInput(-180, 180);
        angleController.setTolerance(0);
        xController.setTolerance(0);
        yController.setTolerance(0);
        xLimiter.reset(0);
        yLimiter.reset(0);

        kinematics = new SwerveDriveKinematics(getModuleTranslations());
        odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(getYaw()), getModulePositions());
    }

    @Override
    public void periodic() {
        for (int i = 0; i < modules.length; i++) {
            modules[i].updateInputs(inputs[i]);
            Logger.getInstance().processInputs("Drive/Module_" + modules[i].getName(), inputs[i]);

            modules[i].setBrakeMode(DriverStation.isEnabled());
        }
        odometry.update(Rotation2d.fromDegrees(getYaw()), getModulePositions());
        Logger.getInstance().recordOutput("Odometry", getPose());
        Logger.getInstance().recordOutput("Drive/TargetStates", lastStates);
        Logger.getInstance().recordOutput("Drive/RealStates", getModuleStates());
    }

    public void setPositionGoal(Pose2d pose) {
        xController.setSetpoint(pose.getX());
        yController.setSetpoint(pose.getY());
        angleController.setGoal(pose.getRotation().getDegrees());
    }

    public ChassisSpeeds calculatePositionalSpeed() {
        double xAdjustment = xController.calculate(getPose().getX());
        double yAdjustment = yController.calculate(getPose().getY());
        double angleAdjustment = angleController.calculate(getYaw());
        return new ChassisSpeeds(xAdjustment, yAdjustment, angleAdjustment);
    }

    public boolean atPositionGoals() {
        return (Math.abs(xController.getPositionError()) < PoseConstants.EPSILON) && (Math.abs(yController.getPositionError()) < PoseConstants.EPSILON) && atAngleGoal();
    }

    public void setAngleGoal(double angle) {
        angleController.setGoal(angle);
    }

    public double calculateAngularSpeed() {
        return angleController.calculate(getYaw());
    }

    public boolean atAngleGoal() {
        return Math.abs(angleController.getPositionError()) < AngleConstants.EPSILON;
    }


    public void drive(double vx, double vy, double degreesPerSecond, boolean fieldOriented) {
        ChassisSpeeds speeds = fieldOriented
                               ? ChassisSpeeds.fromFieldRelativeSpeeds(-vx, -vy, Math.toRadians(degreesPerSecond), getYaw())
                               : new ChassisSpeeds(-vx, -vy, Math.toRadians(degreesPerSecond));

        drive(speeds);
    }

    public void drive(ChassisSpeeds speeds) {
        var states = kinematics.toSwerveModuleStates(speeds);
        setModuleStates(isIdle() ? lastStates : states);
    }

    public void X() {
        var xStates = new SwerveModuleState[] {new SwerveModuleState(0, Rotation2d.fromDegrees(45)), new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), new SwerveModuleState(0, Rotation2d.fromDegrees(45))};
        setModuleStates(xStates);
    }

    public void setModuleStates(SwerveModuleState[] states) {
        for (int i = 0; i < modules.length; i++) modules[i].setState(states[i]);
        lastStates = states;
    }

    public boolean isIdle() {
        boolean isIdle = true;
        for (SwerveModuleState state : getModuleStates()) isIdle = Math.abs(state.speedMetersPerSecond) < 0.001 && isIdle;
        return isIdle;
    }

    public double getYaw() {
        return 0.0;
    }

    public SwerveModulePosition[] getModulePositions() {
        var positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) positions[i] = modules[i].getPosition();
        return positions;
    }

    public SwerveModuleState[] getModuleStates() {
        var states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) states[i] = modules[i].getState();
        return states;
    }

    public Pose2d getPose() {return odometry.getPoseMeters();}
    public void stop() {drive(new ChassisSpeeds(0, 0, 0));}

    public Translation2d[] getModuleTranslations() {
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
