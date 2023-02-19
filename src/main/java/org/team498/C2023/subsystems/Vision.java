package org.team498.C2023.subsystems;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.team498.C2023.FieldPositions;
import org.team498.C2023.Robot;
import org.team498.C2023.RobotState;

import java.util.Optional;

public class Vision extends SubsystemBase {
    private final PhotonCamera cam1 = new PhotonCamera("Arducam_OV9281_USB_Camera");
    private PhotonPipelineResult pipelineResult = new PhotonPipelineResult();
    public Pose2d pose = new Pose2d();
    public PhotonPipelineResult lastResult;
    private final Field2d field = Robot.field;

    private Vision() {
        cam1.setPipelineIndex(0);
        cam1.setDriverMode(false);
    }

    @Override
    public void periodic() {
        pipelineResult = cam1.getLatestResult();
        if (pipelineResult.hasTargets()) {
            lastResult = pipelineResult;
            pose = getPose();
        }
        SmartDashboard.putNumber("Cam X", pose.getX());
        //SmartDashboard.putData(field);
    }

    private Pose2d getPose() {
        Transform3d pose = pipelineResult.getBestTarget().getBestCameraToTarget();
        return new Pose2d(pose.getX(), pose.getY(), /*new Rotation2d(-pose.getRotation().toRotation2d().getRadians())*/ Rotation2d.fromDegrees(Drivetrain.getInstance().getYaw()));
    }

    public int getTargetedTag() {
        if (pipelineResult.hasTargets()) return pipelineResult.getBestTarget().getFiducialId();
        return -1;
    }

    public Optional<Pose2d> getRobotPose() {
        int tag = getTargetedTag();

        if (tag != -1 && tag <= 8) {
            Rotation2d robotAngle = Rotation2d.fromDegrees(Drivetrain.getInstance().getYaw());
            Pose2d pose = new Pose2d(getPose().getX() * robotAngle.getCos() - getPose().getY() * robotAngle.getSin(),
            getPose().getX() * robotAngle.getSin() + getPose().getY() * robotAngle.getCos(), 
            robotAngle).times(-1);

            if (tag > 4) {
                return Optional.of(pose.plus(new Transform2d(new Translation2d(this.pose.getX(), this.pose.getY()), Rotation2d.fromDegrees(180))));
            } else {
                return Optional.of(pose.plus(new Transform2d(new Translation2d(-this.pose.getX(), this.pose.getY()), Rotation2d.fromDegrees(0))));
            }
        }

        return Optional.empty();
    }


    private static Vision instance;

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }
}
