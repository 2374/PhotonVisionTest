package frc.robot.util;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;

public class PhotonCameraWrapper {
    public PhotonCamera camera;
    public PhotonPoseEstimator estimator;
    public AprilTagFieldLayout fieldLayout;
    public PhotonTrackedTarget lastTarget;

    public PhotonCameraWrapper() {
        final AprilTag tag1 = new AprilTag(1, Constants.TAG_1_POSE3D);
        ArrayList<AprilTag> tagList = new ArrayList<>();
        tagList.add(tag1);
        fieldLayout = new AprilTagFieldLayout(tagList, Constants.FIELD_LENGTH,
                Constants.FIELD_WIDTH);
        try {
            fieldLayout = AprilTagFieldLayout
                    .loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            e.printStackTrace();
        }
        camera = new PhotonCamera(Constants.CAMERA_NAME);
        estimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera,
                Constants.ROBOT_TO_CAMERA);
    }

    public PhotonTrackedTarget getBestTarget() {
        PhotonTrackedTarget t = camera.getLatestResult().getBestTarget();
        if (lastTarget != null && t.getPoseAmbiguity() >= .2) {
            return lastTarget;
        }
        return t;
    }

    public AprilTagFieldLayout getFieldLayout() {
        return fieldLayout;
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        estimator.setReferencePose(prevEstimatedRobotPose);
        return estimator.update();
    }
}
