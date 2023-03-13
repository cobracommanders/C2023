package org.team498.C2023.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
import org.team498.lib.util.RotationUtil;
import org.team498.lib.wpilib.ChassisSpeeds;

import static org.team498.C2023.Constants.DrivetrainConstants.*;
import static org.team498.C2023.Ports.Drivetrain.*;

import java.util.function.Supplier;

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
    private final Translation2d[] modulePositions;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private final Gyro gyro = Gyro.getInstance();
    public ChassisSpeeds currentSpeeds = new ChassisSpeeds();

    private final SwerveDriveOdometry fakeOdometry;

    private Drivetrain() {
        WPI_TalonFX FL_Drive = new WPI_TalonFX(FL_DRIVE);
        WPI_TalonFX FR_Drive = new WPI_TalonFX(FR_DRIVE);
        WPI_TalonFX BL_Drive = new WPI_TalonFX(BL_DRIVE);
        WPI_TalonFX BR_Drive = new WPI_TalonFX(BR_DRIVE);
        WPI_TalonFX FL_Steer = new WPI_TalonFX(FL_STEER);
        WPI_TalonFX FR_Steer = new WPI_TalonFX(FR_STEER);
        WPI_TalonFX BL_Steer = new WPI_TalonFX(BL_STEER);
        WPI_TalonFX BR_Steer = new WPI_TalonFX(BR_STEER);
        WPI_CANCoder FL_CANCoder = new WPI_CANCoder(FL_CANCODER);
        WPI_CANCoder FR_CANCoder = new WPI_CANCoder(FR_CANCODER);
        WPI_CANCoder BL_CANCoder = new WPI_CANCoder(BL_CANCODER);
        WPI_CANCoder BR_CANCoder = new WPI_CANCoder(BR_CANCODER);

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
        modulePositions = new Translation2d[] {FL_ModulePosition, FR_ModulePosition, BL_ModulePosition, BR_ModulePosition};
        kinematics = new SwerveDriveKinematics(FL_ModulePosition, FR_ModulePosition, BL_ModulePosition, BR_ModulePosition);
        // odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(0));
        // fakeOdometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(0));

        odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(getYaw()), getModulePositions());
        fakeOdometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(getYaw()), getModulePositions());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pitch", gyro.getPitch());
        SmartDashboard.putNumber("Roll", gyro.getRoll());
        // if (Robot.isReal()) odometry.update(Rotation2d.fromDegrees(getYaw()), getModuleStates());
        if (Robot.isReal()) odometry.update(Rotation2d.fromDegrees(getYaw()), getModulePositions());

        if (RobotState.isDisabled()) {
            for (SwerveModule swerveModule : swerveModules) {
                swerveModule.matchEncoders();
            }
        }

        SmartDashboard.putData(this);
        SmartDashboard.putNumber("Gyro", getYaw());

        Robot.field.setRobotPose(getPose());
		Robot.field.getObject("Swerve Modules").setPoses(getModulePoses());

        SwerveModuleState[] targetStates = getTargetModuleStates();
        SmartDashboard.putNumberArray("Target Swerve States", new double[] {
            targetStates[0].angle.getDegrees(), targetStates[0].speedMetersPerSecond,
            targetStates[1].angle.getDegrees(), targetStates[1].speedMetersPerSecond,
            targetStates[2].angle.getDegrees(), targetStates[2].speedMetersPerSecond,
            targetStates[3].angle.getDegrees(), targetStates[3].speedMetersPerSecond,
        });

        SwerveModuleState[] realStates = getModuleStates();
        SmartDashboard.putNumberArray("Swerve States", new double[] {
            realStates[0].angle.getDegrees(), realStates[0].speedMetersPerSecond,
            realStates[1].angle.getDegrees(), realStates[1].speedMetersPerSecond,
            realStates[2].angle.getDegrees(), realStates[2].speedMetersPerSecond,
            realStates[3].angle.getDegrees(), realStates[3].speedMetersPerSecond,
        });
    }

    @Override
    public void simulationPeriodic() {
        Pose2d currentPose = fakeOdometry.getPoseMeters();

        double newX = currentPose.getX() + currentSpeeds.vxMetersPerSecond * Robot.DEFAULT_PERIOD;
        double newY = currentPose.getY() + currentSpeeds.vyMetersPerSecond * Robot.DEFAULT_PERIOD;
        double newAngle = currentPose.getRotation().getDegrees() + Math.toDegrees(currentSpeeds.omegaRadiansPerSecond * Robot.DEFAULT_PERIOD);

        Robot.field.getObject("Real Pose").setPose(new Pose2d(newX, newY, Rotation2d.fromDegrees(getYaw())));

        // odometry.update(Rotation2d.fromDegrees(getYaw()), getModuleStates());
        odometry.update(Rotation2d.fromDegrees(getYaw()), getModulePositions());
        gyro.setSimAngle(-newAngle);


        // fakeOdometry.resetPosition(new Pose2d(newX, newY, Rotation2d.fromDegrees(newAngle)), Rotation2d.fromDegrees(newAngle));
        fakeOdometry.resetPosition(Rotation2d.fromDegrees(newAngle), getModulePositions(), new Pose2d(newX, newY, Rotation2d.fromDegrees(newAngle)));
    }

    public Pose2d[] getModulePoses() {
        Pose2d[] poses = new Pose2d[swerveModules.length];
        for (int i = 0; i < swerveModules.length; i++) {
            SwerveModule m = swerveModules[i];
            poses[i] = m.getPose(getPose(), modulePositions[i]);
        }

        return poses;
	}

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public Pose2d getNextPose() {
        double x = getPose().getX() + (currentSpeeds.vxMetersPerSecond * (Robot.DEFAULT_PERIOD * 10));
        double y = getPose().getY() + (currentSpeeds.vyMetersPerSecond * (Robot.DEFAULT_PERIOD * 10));
        double rotation = getPose().getRotation().getDegrees() + (currentSpeeds.omegaRadiansPerSecond * (Robot.DEFAULT_PERIOD * 10));
        return new Pose2d(x, y, Rotation2d.fromDegrees(rotation));
    }

    public Supplier<Pose2d> getNextPoseToTag() {
        Pose3d ogPose = Photonvision.getInstance().nearestTagPose().get();
        //TODO: since this is the tag pose, it might need to subtract the current velocity
        double x = ogPose.getX() + (currentSpeeds.vxMetersPerSecond * (Robot.DEFAULT_PERIOD * 10));
        double y = ogPose.getY() + (currentSpeeds.vyMetersPerSecond * (Robot.DEFAULT_PERIOD * 10));
        double rotation = ogPose.getRotation().toRotation2d().getDegrees() + (currentSpeeds.omegaRadiansPerSecond * (Robot.DEFAULT_PERIOD * 10));
        return () -> new Pose2d(x, y, Rotation2d.fromDegrees(rotation));
    }

    public Transform2d getVelocity() {
        return new Transform2d(new Translation2d(currentSpeeds.vxMetersPerSecond * (Robot.DEFAULT_PERIOD * 10), currentSpeeds.vyMetersPerSecond * (Robot.DEFAULT_PERIOD * 10)), Rotation2d.fromRadians(currentSpeeds.omegaRadiansPerSecond * (Robot.DEFAULT_PERIOD * 10)));
    }

    public double getNextDistanceToTag() {
        return Math.abs(getNextPoseToTag().get().getTranslation().getNorm());
    }

    public void setPose(Pose2d pose) {
        // odometry.resetPosition(pose, Rotation2d.fromDegrees(gyro.getYaw()));
        // fakeOdometry.resetPosition(pose, Rotation2d.fromDegrees(gyro.getYaw()));
        odometry.resetPosition(Rotation2d.fromDegrees(getYaw()), getModulePositions(), pose);
        fakeOdometry.resetPosition(Rotation2d.fromDegrees(getYaw()), getModulePositions(), pose);
        // gyro.setAngleOffset(pose.getRotation().getDegrees());
        gyro.setYaw(pose.getRotation().getDegrees());
    }

    public void setOdometry(Pose3d pose) {
        // odometry.resetPosition(new Pose2d(pose.getX(), pose.getY(), Rotation2d.fromDegrees(gyro.getYaw())), Rotation2d.fromDegrees(gyro.getYaw()));
        // fakeOdometry.resetPosition(new Pose2d(pose.getX(), pose.getY(), Rotation2d.fromDegrees(gyro.getYaw())), Rotation2d.fromDegrees(gyro.getYaw()));
        odometry.resetPosition(Rotation2d.fromDegrees(gyro.getYaw()), getModulePositions(), new Pose2d(pose.getX(), pose.getY(), Rotation2d.fromDegrees(gyro.getYaw())));
        fakeOdometry.resetPosition(Rotation2d.fromDegrees(gyro.getYaw()), getModulePositions(), new Pose2d(pose.getX(), pose.getY(), Rotation2d.fromDegrees(gyro.getYaw())));
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
        return angleController.calculate(getYaw());
    }

    /** @return true if the angle controller is at its goal */
    public boolean atAngleGoal() {return Math.abs(angleController.getPositionError()) < angleController.getPositionTolerance();}

    /** Sets the swerve drive to move towards the values specified by the position controllers. */
    public void driveToPositionGoals() {
        double xAdjustment = xController.calculate(getPose().getX());
        double yAdjustment = yController.calculate(getPose().getY());
        double angleAdjustment = angleController.calculate(getYaw());
        drive(xAdjustment, yAdjustment, angleAdjustment, true);
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
                               ? ChassisSpeeds.fromFieldRelativeSpeeds(-vx, -vy, Math.toRadians(degreesPerSecond), getYaw())
                               : new ChassisSpeeds(-vx, -vy, Math.toRadians(degreesPerSecond));

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

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] positions = new SwerveModuleState[swerveModules.length];
        for (int i = 0; i < swerveModules.length; i++) {
            positions[i] = swerveModules[i].getState();
        }
        return positions;
    }

    public SwerveModuleState[] getTargetModuleStates() {
        SwerveModuleState[] positions = new SwerveModuleState[swerveModules.length];
        for (int i = 0; i < swerveModules.length; i++) {
            positions[i] = swerveModules[i].getTargetState();
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

    public double distanceTo(Point point) {
        double x = point.getX();
        double y = point.getY();

        double xDiff = x - getPose().getX();
        double yDiff = y - getPose().getY();

        return Math.hypot(xDiff, yDiff);
    }

    public double calculateDegreesToTarget(Pose2d target) {
        Pose2d currentPose = getPose();

        // Estimate the future pose of the robot to compensate for lag
        double newX = currentPose.getX() + (currentSpeeds.vxMetersPerSecond * (Robot.DEFAULT_PERIOD * (Robot.isReal() ? 10 : 0)));
        double newY = currentPose.getY() + (currentSpeeds.vyMetersPerSecond * (Robot.DEFAULT_PERIOD * (Robot.isReal() ? 10 : 0)));

        Pose2d futurePose = new Pose2d(newX, newY, new Rotation2d());

        // Calculate the angle between the target and the current robot position.
        double angle = Math.toDegrees(Math.atan2(-futurePose.getY() + target.getY(), -futurePose.getX() + target.getX()));

        // Normalize the angle to a number between 0 and 360.
        angle = RotationUtil.toSignedDegrees(angle);

        // Return the angle to which the turret needs to be adjusted.
        return angle;
    }


    private static Drivetrain instance;

    public static Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain();
        }
        return instance;
    }

}
