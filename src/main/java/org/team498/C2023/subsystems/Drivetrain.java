package org.team498.C2023.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team498.C2023.Robot;
import org.team498.C2023.Constants.DrivetrainConstants;
import org.team498.lib.drivers.Gyro;
import org.team498.lib.drivers.SwerveModule;
import org.team498.lib.field.BaseRegion;
import org.team498.lib.field.Point;
import org.team498.lib.wpilib.ChassisSpeeds;

import static org.team498.C2023.Constants.DrivetrainConstants.*;
import static org.team498.C2023.Ports.Drivetrain.*;

public class Drivetrain extends SubsystemBase {
    // Profiled controller for the rotation of the robot
    private final ProfiledPIDController angleController = new ProfiledPIDController(AngleConstants.P,
                                                                                    AngleConstants.I,
                                                                                    AngleConstants.D,
                                                                                    AngleConstants.CONTROLLER_CONSTRAINTS
    );
    // Profiled controller for the x position of the robot
    private final PIDController xController = new PIDController(PoseConstants.P, PoseConstants.I, PoseConstants.D);
    private static SlewRateLimiter xLimiter = new SlewRateLimiter(DrivetrainConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    // Profiled controller for the y position of the robot
    private final PIDController yController = new PIDController(PoseConstants.P, PoseConstants.I, PoseConstants.D);
    private static SlewRateLimiter yLimiter = new SlewRateLimiter(DrivetrainConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    private final SwerveModule[] swerveModules;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private final Gyro gyro = Gyro.getInstance();
    public ChassisSpeeds currentSpeeds = new ChassisSpeeds();

    private Drivetrain() {
        TalonFX FL_Drive = new TalonFX(FL_DRIVE);
        TalonFX FR_Drive = new TalonFX(FR_DRIVE);
        TalonFX BL_Drive = new TalonFX(BL_DRIVE);
        TalonFX BR_Drive = new TalonFX(BR_DRIVE);
        TalonFX FL_Steer = new TalonFX(FL_STEER);
        TalonFX FR_Steer = new TalonFX(FR_STEER);
        TalonFX BL_Steer = new TalonFX(BL_STEER);
        TalonFX BR_Steer = new TalonFX(BR_STEER);
        CANCoder FL_CANCoder = new CANCoder(FL_CANCODER);
        CANCoder FR_CANCoder = new CANCoder(FR_CANCODER);
        CANCoder BL_CANCoder = new CANCoder(BL_CANCODER);
        CANCoder BR_CANCoder = new CANCoder(BR_CANCODER);

        SwerveModule FL_Module = new SwerveModule("FL", FL_Drive, FL_Steer, FL_CANCoder, FL_MODULE_OFFSET);
        SwerveModule FR_Module = new SwerveModule("FR", FR_Drive, FR_Steer, FR_CANCoder, FR_MODULE_OFFSET);
        SwerveModule BL_Module = new SwerveModule("BL", BL_Drive, BL_Steer, BL_CANCoder, BL_MODULE_OFFSET);
        SwerveModule BR_Module = new SwerveModule("BR", BR_Drive, BR_Steer, BR_CANCoder, BR_MODULE_OFFSET);

        // Put all the swerve modules in an array to make using them easier
        swerveModules = new SwerveModule[] {FL_Module, FR_Module, BL_Module, BR_Module};

        angleController.enableContinuousInput(-180, 180);
        angleController.setTolerance(AngleConstants.EPSILON);
        xController.setTolerance(0);
        yController.setTolerance(0);
        xLimiter.reset(0);
        yLimiter.reset(0);

        // Set up the kinematics
        double moduleDistance = Units.inchesToMeters(SWERVE_MODULE_DISTANCE_FROM_CENTER);
        Translation2d FL_ModulePosition = new Translation2d(moduleDistance, moduleDistance);
        Translation2d FR_ModulePosition = new Translation2d(moduleDistance, -moduleDistance);
        Translation2d BL_ModulePosition = new Translation2d(-moduleDistance, moduleDistance);
        Translation2d BR_ModulePosition = new Translation2d(-moduleDistance, -moduleDistance);
        kinematics = new SwerveDriveKinematics(FL_ModulePosition, FR_ModulePosition, BL_ModulePosition, BR_ModulePosition);
        odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(0), getModulePositions());
    }

    @Override
    public void periodic() {
        if (Robot.isReal()) odometry.update(Rotation2d.fromDegrees(getYaw()+180), getModulePositions());

        if (RobotState.isDisabled()) {
            for (SwerveModule swerveModule : swerveModules) {
                swerveModule.matchEncoders();
            }
        }

        SmartDashboard.putData(this);
        SmartDashboard.putNumber("Gyro", getYaw());

        Robot.field.setRobotPose(getPose());
    }

    @Override
    public void simulationPeriodic() {
        Pose2d currentPose = getPose();

        double newX = currentPose.getX() + currentSpeeds.vxMetersPerSecond * Robot.kDefaultPeriod;
        double newY = currentPose.getY() + currentSpeeds.vyMetersPerSecond * Robot.kDefaultPeriod;
        double newAngle = currentPose.getRotation().getDegrees() + Math.toDegrees(currentSpeeds.omegaRadiansPerSecond * Robot.kDefaultPeriod);
        gyro.setSimAngle(newAngle);

        Robot.field.setRobotPose(newX, newY, Rotation2d.fromDegrees(-newAngle));

        odometry.resetPosition(Rotation2d.fromDegrees(newAngle), getModulePositions(), new Pose2d(newX, newY, Rotation2d.fromDegrees(newAngle)));
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public Pose2d getNextPose() {
        double x = getPose().getX() + (currentSpeeds.vxMetersPerSecond * (Robot.kDefaultPeriod * 10));
        double y = getPose().getY() + (currentSpeeds.vyMetersPerSecond * (Robot.kDefaultPeriod * 10));
        double rotation = getPose().getRotation().getDegrees() + (currentSpeeds.omegaRadiansPerSecond * (Robot.kDefaultPeriod * 10));
        return new Pose2d(x, y, Rotation2d.fromDegrees(rotation));
    }

    public void setPose(Pose2d pose) {
        odometry.resetPosition(Rotation2d.fromDegrees(gyro.getYaw()), getModulePositions(), pose);
        gyro.setAngleOffset(pose.getRotation().getDegrees());
    }

    public void setOdometry(Pose3d pose) {
        odometry.resetPosition(Rotation2d.fromDegrees(gyro.getYaw()),
                               getModulePositions(),
                               new Pose2d(pose.getX(), pose.getY(), Rotation2d.fromDegrees(gyro.getYaw()))
        );
    }

    /** @return true if all three swerve controllers have reached their position goals (x pos, y pos, angle) */
    public boolean atPositionGoals() {
        return (Math.abs(xController.getPositionError()) < PoseConstants.EPSILON) && (Math.abs(yController.getPositionError()) < PoseConstants.EPSILON) && atAngleGoal();
    }

    /** Sets the position goals of the swerve drive. */
    public void setPositionGoal(Pose2d pose2d) {
        xController.setSetpoint(pose2d.getX());
        yController.setSetpoint(pose2d.getY());
        angleController.setGoal(pose2d.getRotation().getDegrees());
    }

    /** Sets the goal of the angle controller to a specified target in degrees. */
    public void setAngleGoal(double goal) {
        SmartDashboard.putNumber("Angle Goal", goal);
        angleController.setGoal(goal);
    }

    /** Calculate the rotational speed from the pid controller */
    public double calculateRotationalSpeed() {
        return -angleController.calculate(getYaw());
    }

    /** @return true if the angle controller is at its goal */
    public boolean atAngleGoal() {return Math.abs(angleController.getPositionError()) < angleController.getPositionTolerance();}

    /** Sets the swerve drive to move towards the values specified by the position controllers. */
    public void driveToPositionGoals() {
        double xAdjustment = xController.calculate(getPose().getX());
        double yAdjustment = yController.calculate(getPose().getY());
        double angleAdjustment = -angleController.calculate(getYaw());
        drive(xAdjustment, yAdjustment, angleAdjustment * Robot.rotationFlip, true);
    }

    /**
     * Sets the swerve drive to desired speed of direction and rotation.
     *
     * @param vx               x velocity
     * @param vy               y velocity
     * @param degreesPerSecond rotational speed in degrees per second
     * @param fieldOriented    true if the robot is driving in field oriented mode, false if robot oriented
     */
    public void drive(double vx, double vy, double degreesPerSecond, boolean fieldOriented) {
        ChassisSpeeds speeds = fieldOriented
                               ? ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, Math.toRadians(degreesPerSecond), getYaw())
                               : new ChassisSpeeds(vx, vy, Math.toRadians(degreesPerSecond));

        drive(speeds, new Translation2d());
    }

    public void stop() {drive(0, 0, 0, false);}

    /**
     * Sets the swerve drive to desired speed of direction and rotation, with the option to use a custom center of rotation.
     *
     * @param chassisSpeeds drive speeds to set
     * @param rotation      a {@link Translation2d} representing the distance from the center of the robot to the desired center of rotation
     */
    public void drive(ChassisSpeeds chassisSpeeds, Translation2d rotation) {
        ChassisSpeeds limited = limitChassisSpeeds(chassisSpeeds);
        setModuleStates(kinematics.toSwerveModuleStates(limited, rotation), false);
    }

    /**
     * Set the swerve modules to the desired states. Under normal operation, the angle will not change unless the robot is moving, however this can be
     * overridden by setting 'force' to true.
     *
     * @param moduleStates an array of {@link SwerveModuleState module states} to set the swerve to
     * @param force        if set to true, the module states will be set even if the robot is not moving
     */
    public void setModuleStates(SwerveModuleState[] moduleStates, boolean force) {
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_VELOCITY_METERS_PER_SECOND);

        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setState(moduleStates[i], force);
        }
        
        currentSpeeds = ChassisSpeeds.toFieldRelativeSpeeds(kinematics.toChassisSpeeds(moduleStates), Rotation2d.fromDegrees(getYaw()));
    }

    /** @return an array of {@link edu.wpi.first.math.kinematics.SwerveModulePosition module positions} representing each of the modules */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[swerveModules.length];
        for (int i = 0; i < swerveModules.length; i++) {
            positions[i] = swerveModules[i].getPosition();
        }
        return positions;
    }

    /** @return The current yaw angle in degrees (-180 to 180) */
    public double getYaw() {
        return gyro.getYaw();
    }

    /**
     * Checks if the drivetrain is in a region on the field
     *
     * @param region the region to check
     * @return true if in the region
     */
    public boolean isInRegion(BaseRegion region) {
        return region.contains(Point.fromPose2d(getPose()));
    }

    /**
     * Checks if the drivetrain is near a pose on the field
     *
     * @param pose    the pose to test
     * @param epsilon maximum distance to the pose
     * @return true if distance to the pose is less than the provided epsilon
     */
    public boolean isNear(Pose2d pose, double epsilon) {
        return Math.hypot(getPose().getX() - pose.getX(), getPose().getY() - pose.getY()) < epsilon;
    }
    public static ChassisSpeeds limitChassisSpeeds(ChassisSpeeds unlimited) {
        double x = xLimiter.calculate(unlimited.vxMetersPerSecond);
        double y = yLimiter.calculate(unlimited.vyMetersPerSecond);
        double r = unlimited.omegaRadiansPerSecond;
        return new ChassisSpeeds(x, y, r);
    }


    private static Drivetrain instance;

    public static Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain();
        }
        return instance;
    }

}
