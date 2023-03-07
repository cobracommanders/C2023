package org.team498.C2023;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team498.C2023.subsystems.Drivetrain;

public class RobotState extends SubsystemBase {
    private final Drivetrain drivetrain;
    private GamePiece currentGameMode = GamePiece.CONE;
    private State state = State.IDLE_CUBE;
    private State nextDriveteamState = State.IDLE_CUBE;

    public enum GamePiece {
        CUBE,
        CONE
    }

    private RobotState() {
        this.drivetrain = Drivetrain.getInstance();
    }

    public void setCurrentGameMode(GamePiece gamePiece) {
        currentGameMode = gamePiece;
        SmartDashboard.putBoolean("Current Game Piece", gamePiece == GamePiece.CONE);
    }

    public State getCurrentState() {
        return state;
    }

    public void setState(State state) {
        this.state = state;
    }

    public State getNextDriveteamState() {
        return nextDriveteamState;
    }

    public void setNextDriveteamState(State nextTargetState) {
        this.nextDriveteamState = nextTargetState;
    }

    public boolean inCubeMode() {return currentGameMode == GamePiece.CUBE;}
    public boolean inConeMode() {return currentGameMode == GamePiece.CONE;}

    public Transform2d getRobotToField() {
        return toTransform2d(drivetrain.getPose());
    }

    // public Transform2d getRobotToTarget() {
    //     return getVisionToTarget().plus(getVisionToRobot().inverse());
    // }

    public Transform2d getRobotToPoint(Pose2d target) {
        return new Transform2d(drivetrain.getPose(), target);
    }

    // public Transform2d getVisionToTarget() {
    //     return toTransform2d(vision.getEstimatedGlobalPose(drivetrain.getPose()).get().estimatedPose);
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
