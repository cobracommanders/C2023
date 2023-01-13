package org.team498.C2023.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team498.C2023.Constants.PoseConstants;
import org.team498.C2023.Constants.SnapConstants;
import org.team498.C2023.Robot;
import org.team498.lib.drivers.Gyro;
import org.team498.lib.drivers.SwerveModule;
import org.team498.lib.wpilib.ChassisSpeeds;

import static org.team498.C2023.Constants.DrivetrainConstants.*;
import static org.team498.C2023.Ports.*;

public class Drivetrain extends SubsystemBase {
    // Profiled controller for the rotation of the robot
    // TODO see if the profiled controller here causes the robot to turn on startup
    private final ProfiledPIDController angleController = new ProfiledPIDController(SnapConstants.P, SnapConstants.I, SnapConstants.D, SnapConstants.CONTROLLER_CONSTRAINTS);
    // Profiled controller for the x position of the robot
    private final PIDController xController = new PIDController(PoseConstants.P, PoseConstants.I, PoseConstants.D);
    // Profiled controller for the y position of the robot
    private final PIDController yController = new PIDController(PoseConstants.P, PoseConstants.I, PoseConstants.D);

    // Holonomic drive controller to follow trajectories
    private final HolonomicDriveController trajectoryController = new HolonomicDriveController(new PIDController(PoseConstants.P, PoseConstants.I, PoseConstants.D), new PIDController(PoseConstants.P, PoseConstants.I, PoseConstants.D), angleController);
    private final SwerveModule[] swerveModules;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private final Gyro gyro = Gyro.getInstance();

    private Drivetrain() {
        TalonFX FL_Drive = new TalonFX(FL_DRIVE_ID);
        TalonFX FR_Drive = new TalonFX(FR_DRIVE_ID);
        TalonFX BL_Drive = new TalonFX(BL_DRIVE_ID);
        TalonFX BR_Drive = new TalonFX(BR_DRIVE_ID);
        TalonFX FL_Steer = new TalonFX(FL_STEER_ID);
        TalonFX FR_Steer = new TalonFX(FR_STEER_ID);
        TalonFX BL_Steer = new TalonFX(BL_STEER_ID);
        TalonFX BR_Steer = new TalonFX(BR_STEER_ID);
        CANCoder FL_CANCoder = new CANCoder(FL_CANCODER_ID);
        CANCoder FR_CANCoder = new CANCoder(FR_CANCODER_ID);
        CANCoder BL_CANCoder = new CANCoder(BL_CANCODER_ID);
        CANCoder BR_CANCoder = new CANCoder(BR_CANCODER_ID);

        SwerveModule FL_Module = new SwerveModule(FL_Drive, FL_Steer, FL_CANCoder, FL_MODULE_OFFSET);
        SwerveModule FR_Module = new SwerveModule(FR_Drive, FR_Steer, FR_CANCoder, FR_MODULE_OFFSET);
        SwerveModule BL_Module = new SwerveModule(BL_Drive, BL_Steer, BL_CANCoder, BL_MODULE_OFFSET);
        SwerveModule BR_Module = new SwerveModule(BR_Drive, BR_Steer, BR_CANCoder, BR_MODULE_OFFSET);

        // Put all the swerve modules in an array to make using them easier
        swerveModules = new SwerveModule[] {FL_Module, FR_Module, BL_Module, BR_Module};

        angleController.enableContinuousInput(-180, 180);
        angleController.setTolerance(SnapConstants.EPSILON);
        xController.setTolerance(PoseConstants.EPSILON);
        yController.setTolerance(PoseConstants.EPSILON);
        trajectoryController.setTolerance(new Pose2d(PoseConstants.EPSILON, PoseConstants.EPSILON, Rotation2d.fromDegrees(SnapConstants.EPSILON)));

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
        if (Robot.isReal()) odometry.update(Rotation2d.fromDegrees(-getYaw()), getModulePositions());

        if (RobotState.isDisabled()) {
            matchEncoders();
        }

        SmartDashboard.putData(this);
        SmartDashboard.putNumber("Gyro", getYaw());
        SmartDashboard.putNumber("Snap Setpoint", angleController.getGoal().position);

        SmartDashboard.putNumber("Goal X", xController.getSetpoint());
        SmartDashboard.putNumber("Goal Y", yController.getSetpoint());
        SmartDashboard.putNumber("Pose X", getPose().getX());
        SmartDashboard.putNumber("Pose Y", getPose().getY());

    }

    public ChassisSpeeds currentSpeeds = new ChassisSpeeds();



    @Override
    public void simulationPeriodic() {
        Pose2d oldPose = getPose();

        double newX = oldPose.getX() + currentSpeeds.vxMetersPerSecond * Robot.kDefaultPeriod;
        double newY = oldPose.getY() + currentSpeeds.vyMetersPerSecond * Robot.kDefaultPeriod;
        double newAngle = oldPose.getRotation().getDegrees() + Math.toDegrees(currentSpeeds.omegaRadiansPerSecond * Robot.kDefaultPeriod);
        gyro.setSimAngle(newAngle);

        Robot.field.setRobotPose(newX, newY, Rotation2d.fromDegrees(-newAngle));

        odometry.resetPosition(Rotation2d.fromDegrees(newAngle), getModulePositions(), new Pose2d(newX, newY, Rotation2d.fromDegrees(newAngle)));

    }

    public ChassisSpeeds getSpeedsFromTrajectoryState(Trajectory.State goal) {
        return (ChassisSpeeds) trajectoryController.calculate(getPose(), goal, goal.poseMeters.getRotation());
    }

    public boolean atTrajectoryGoal() {
        return trajectoryController.atReference();
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void setInitialPose(Pose2d pose) {
        odometry.resetPosition(Rotation2d.fromDegrees(gyro.getAngle()), getModulePositions(), pose);
        gyro.setAngleOffset(pose.getRotation().getDegrees());
    }

    /**
     * @return true if all three swerve controllers have reached their position goals (x pos, y pos, angle)
     */
    public boolean atPositionGoals() {
        return xController.atSetpoint() && yController.atSetpoint() && angleController.atGoal();
    }

    /**
     * Sets the position goals of the swerve drive.
     *
     * @param pose2d The position goal
     */
    public void setPositionGoal(Pose2d pose2d) {
        xController.setSetpoint(pose2d.getX());
        yController.setSetpoint(pose2d.getY());
        angleController.setGoal(pose2d.getRotation().getDegrees());
    }

    /**
     * @return the value from the x position controller with a custom input measurement
     */
    public double calculateXController(double measurement) {
        return xController.calculate(measurement);
    }

    /**
     * @return the value from the y position controller with a custom input measurement
     */
    public double calculateYController(double measurement) {
        return yController.calculate(measurement);
    }

    /**
     * @return the value from the snap controller with a custom input measurement
     */
    public double calculateSnapController(double measurement) {
        return angleController.calculate(measurement);
    }

    /**
     * Sets the goal of the snap controller to a specified target.
     *
     * @param goal the goal to set in degrees
     */
    public void setSnapGoal(double goal) {
        angleController.setGoal(goal);
    }

    /**
     * Calculate the rotational speed from the pid controller, unless it's already at the goal
     */
    public double calculateSnapSpeed() {
        return angleController.atGoal()
               ? 0
               : -angleController.calculate(getYaw());
    }

    /** @return true if the snap controller is at its goal */
    public boolean atSnapGoal() {
        return angleController.atGoal();
    }

    /**
     * Sets the swerve drive to move towards the values specified by the position controllers.
     */
    public void driveToPositionGoals() {
        // double xAdjustment = xController.calculate(getPose().getY());
        // double yAdjustment = -yController.calculate(getPose().getX());
        // double angleAdjustment = angleController.calculate(getYaw());

        double xAdjustment = xController.calculate(getPose().getX());
        double yAdjustment = yController.calculate(getPose().getY());
        double angleAdjustment = -angleController.calculate(getYaw());
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
                               ? ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, Math.toRadians(degreesPerSecond), getYaw())
                               : new ChassisSpeeds(vx, vy, Math.toRadians(degreesPerSecond));

        drive(speeds, new Translation2d());
    }

    public void stop() {
        drive(0, 0, 0, false);
    }

    /**
     * Sets the swerve drive to desired speed of direction and rotation, with the option to use a custom center of rotation.
     *
     * @param chassisSpeeds drive speeds to set
     * @param rotation      a {@link Translation2d} representing the distance from the center of the robot to the desired center of rotation
     */
    public void drive(ChassisSpeeds chassisSpeeds, Translation2d rotation) {
        // Use the kinematics to set the desired speed and angle for each swerve module
        // using the input velocities for direction and rotation
        setModuleStates(kinematics.toSwerveModuleStates(chassisSpeeds, rotation), false);
    }

    public void matchEncoders() {
        for (SwerveModule swerveModule : swerveModules) {
            swerveModule.matchEncoders();
        }
    }

    /**
     * Set the swerve modules to the desired states. Under normal operation, the angle will not change unless the robot is moving, however this can be overridden by setting 'force' to true.
     *
     * @param moduleStates an array of {@link SwerveModuleState module states} to set the swerve to
     * @param force        if set to true, the module states will be set even if the robot is not moving
     */
    public void setModuleStates(SwerveModuleState[] moduleStates, boolean force) {
        // SwerveModuleState[] moduleStates =
        // kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(kinematics.toChassisSpeeds(states),
        // Rotation2d.fromDegrees(getYaw())));

        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_VELOCITY_METERS_PER_SECOND);

        // Set the motors of the swerve module to the calculated state
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setState(moduleStates[i], force);
        }

        currentSpeeds = ChassisSpeeds.toFieldRelativeSpeeds(kinematics.toChassisSpeeds(moduleStates), Rotation2d.fromDegrees(getYaw()));
    }

    /**
     * @return an array of {@link edu.wpi.first.math.kinematics.SwerveModuleState module states} representing each of the modules
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[swerveModules.length];
        for (int i = 0; i < swerveModules.length; i++) {
            states[i] = swerveModules[i].getState();
        }
        return states;
    }

    /**
     * @return an array of {@link edu.wpi.first.math.kinematics.SwerveModulePosition module positions} representing each of the modules
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[swerveModules.length];
        for (int i = 0; i < swerveModules.length; i++) {
            positions[i] = swerveModules[i].getPosition();
        }
        return positions;
    }

    /** @return The current yaw angle in degrees (-180 to 180) */
    public double getYaw() {
        return gyro.getAngle();
    }

    public void resetAngleController() {
        angleController.reset(getYaw());
    }

    public PPSwerveControllerCommand getPathPlannerCommand(PathPlannerTrajectory trajectory) {
        setInitialPose(trajectory.getInitialHolonomicPose());
        return new PPSwerveControllerCommand(trajectory, this::getPose, // Pose supplier
                                             kinematics, // SwerveDriveKinematics
                                             xController, // X controller. Tune these values for your robot. Leaving them 0 will only use
                                             // feedforwards.
                                             yController, // Y controller (usually the same values as X controller)
                                             new PIDController(SnapConstants.P, SnapConstants.I, SnapConstants.D), // Rotation controller. Tune these
                                             // values for your robot. Leaving
                                             // them 0 will only use
                                             // feedforwards.
                                             k -> setModuleStates(k, false), // Module states consumer
                                             this // Requires this drive subsystem
        );
    }

    private static Drivetrain instance;

    public static Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain();
        }
        return instance;
    }

}
