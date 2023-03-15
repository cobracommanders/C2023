package org.team498.C2023;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team498.C2023.subsystems.Drivetrain;

public class RobotState extends SubsystemBase {
    private final Drivetrain drivetrain;
    private GameMode currentGameMode = GameMode.CONE;
    private ScoringOption nextScoringOption = ScoringOption.MID;

    private State state = State.IDLE_CUBE;

    public enum GameMode {
        CUBE,
        CONE
    }

    public enum ScoringOption {
        TOP, MID, SPIT
    }

    private boolean shootDrive = false;

    private RobotState() {
        this.drivetrain = Drivetrain.getInstance();
    }

    public void setCurrentGameMode(GameMode gameMode) {
        currentGameMode = gameMode;
        SmartDashboard.putBoolean("Current Game Piece", gameMode == GameMode.CONE);
    }

    public State getCurrentState() {
        return state;
    }

    public void setState(State state) {
        this.state = state;
    }

    public ScoringOption getNextScoringOption() {
        return nextScoringOption;
    }

    public void setNextScoringOption(ScoringOption nextScoringOption) {
        this.nextScoringOption = nextScoringOption;
    }

    public void setShootDrive(boolean shootDrive) {
        this.shootDrive = shootDrive;
    }

    public boolean inShootDriveMode() {
        return shootDrive;
    }

    public State getNextScoringState() {
        State state;

        if (shootDrive) {
            state = switch (nextScoringOption) {
                case TOP -> State.SHOOT_DRIVE_CUBE_TOP;
                case MID -> currentGameMode == GameMode.CONE ? State.SHOOT_DRIVE_CONE_MID : State.SHOOT_DRIVE_CUBE_MID;
                case SPIT -> State.SPIT_CUBE;
            };
        } else {
            state = switch (nextScoringOption) {
                case TOP -> currentGameMode == GameMode.CONE ? State.TOP_CONE : State.TOP_CUBE;
                case MID -> currentGameMode == GameMode.CONE ? State.MID_CONE : State.MID_CUBE;
                case SPIT -> State.SPIT_CUBE;
            };
        }

        SmartDashboard.putString("Driveteam State", state.name());
        return state;
    }

    public boolean inCubeMode() {
        return currentGameMode == GameMode.CUBE;
    }

    public boolean inConeMode() {
        return currentGameMode == GameMode.CONE;
    }

    public Transform2d getRobotToField() {
        return toTransform2d(drivetrain.getPose());
    }

    // public Transform2d getRobotToTarget() {
    // return getVisionToTarget().plus(getVisionToRobot().inverse());
    // }

    public Transform2d getRobotToPoint(Pose2d target) {
        return new Transform2d(drivetrain.getPose(), target);
    }

    // public Transform2d getVisionToTarget() {
    // return
    // toTransform2d(vision.getEstimatedGlobalPose(drivetrain.getPose()).get().estimatedPose);
    // }

    public Transform2d getVisionToRobot() {
        return new Transform2d();
    }

    public Transform2d getVisionToField() {
        return getRobotToField().plus(getVisionToRobot());
    }

    public Transform2d toTransform2d(Pose2d pose) {
        return new Transform2d(new Pose2d(), pose);
    }

    public Pose2d toPose2d(Transform2d pose) {
        return new Pose2d(pose.getX(), pose.getY(), pose.getRotation());
    }

    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }
}
