package org.team498.C2023.subsystems;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.team498.C2023.FieldPositions;

import java.util.Optional;

public class Vision extends SubsystemBase {
    private final PhotonCamera cam1 = new PhotonCamera("Arducam_OV9281_USB_Camera");
    private PhotonPipelineResult pipelineResult = new PhotonPipelineResult();
    public Pose2d pose = new Pose2d();
    public PhotonPipelineResult lastResult;
    private final Field2d field = new Field2d();

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
        SmartDashboard.putData(field);
    }

    private Pose2d getPose() {
        Transform3d pose = pipelineResult.getBestTarget().getBestCameraToTarget();
        double y = -pose.getX() / Math.tan(Math.PI / 2 - pose.getRotation().getZ());
        return new Pose2d(pose.getX(), y, new Rotation2d(-pose.getRotation().toRotation2d().getRadians()));
    }

    public int getTargetedTag() {
        if (pipelineResult.hasTargets()) return pipelineResult.getBestTarget().getFiducialId();
        return -1;
    }

    public Optional<Pose2d> getRobotPose() {
        int tag = getTargetedTag();

        if (tag != -1 && tag <= 8) {            
            Pose2d pose = new Pose2d(FieldPositions.aprilTags.get(tag).getX(), FieldPositions.aprilTags.get(tag).getY(), new Rotation2d());

            if (tag > 4) {
                return Optional.of(pose.plus(new Transform2d(new Translation2d(this.pose.getX(), this.pose.getY()), new Rotation2d())));
            } else {
                return Optional.of(pose.plus(new Transform2d(new Translation2d(-this.pose.getX(), this.pose.getY()), Rotation2d.fromDegrees(180))));
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
