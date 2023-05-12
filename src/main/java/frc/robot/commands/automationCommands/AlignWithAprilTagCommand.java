package frc.robot.commands.automationCommands;

import java.util.Optional;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.common.control.SimplePathBuilder;
import frc.common.control.Trajectory;
import frc.common.math.Rotation2;
import frc.common.math.Vector2;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AutonomousTrajectories;

public class AlignWithAprilTagCommand extends CommandBase {

    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final int targetID;

    public AlignWithAprilTagCommand(DrivetrainSubsystem drivetrainSubsystem, int t) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        targetID = t;
        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void initialize() { // use pose from robot to target to align.
        // AprilTag target =
        // m_drivetrainSubsystem.getPhotonCameraWrapper().getFieldLayout().getTags()
        // .get(m_drivetrainSubsystem.getPhotonCameraWrapper().getBestTarget().getFiducialId()
        // - 1); // only works
        // // if all tags
        // // exist on
        // // field
        // double targetAngle = Math.toDegrees(target.pose.getRotation().getAngle());
        // m_drivetrainSubsystem.getFollower().follow(new Trajectory(
        // new SimplePathBuilder(
        // new Vector2(m_drivetrainSubsystem.getPose().getX(),
        // m_drivetrainSubsystem.getPose().getY()),
        // Rotation2.fromDegrees(m_drivetrainSubsystem.getPose().getRotation().getDegrees()))
        // .lineTo(new Vector2(target.pose.getX() + Math.cos(targetAngle),
        // target.pose.getY() + Math.sin(targetAngle) + -.5), // 1 meter away from face
        // Rotation2.fromDegrees(targetAngle + 180))
        // .build(),
        // DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS, 0));

        // PhotonTrackedTarget target =
        // m_drivetrainSubsystem.getPhotonCameraWrapper().getBestTarget();
        // m_drivetrainSubsystem.getFollower().follow(new Trajectory(
        // new SimplePathBuilder(new Vector2(0, 0), Rotation2.fromDegrees(0)).lineTo(
        // Vector2.translation2dToVector2(
        // target.getBestCameraToTarget().plus(Constants.ROBOT_TO_CAMERA.inverse()).getTranslation().toTranslation2d()),
        // Rotation2.fromDegrees(0)).build(),
        // DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS,
        // AutonomousTrajectories.SAMPLE_DISTANCE));

        // Optional<EstimatedRobotPose> pose =
        // m_drivetrainSubsystem.getPhotonCameraWrapper()
        // .getEstimatedGlobalPose(m_drivetrainSubsystem.getPose());
        PhotonPipelineResult targets = m_drivetrainSubsystem.getPhotonCameraWrapper().camera.getLatestResult();
        if (targets.hasTargets()) {
            Optional<PhotonTrackedTarget> target = targets.getTargets().stream()
                    .filter(t -> t.getFiducialId() == targetID).findFirst(); // get correct target
            if (target.isPresent()) {
                PhotonTrackedTarget tar = target.get();
                Transform3d camToTarget = tar.getBestCameraToTarget(); // get transfor for it
                Pose2d finalPose = new Pose2d()
                        .transformBy(
                                new Transform2d(Constants.ROBOT_TO_CAMERA.getTranslation().toTranslation2d(),
                                        Constants.ROBOT_TO_CAMERA.getRotation().toRotation2d())) // center to robot
                        .transformBy(new Transform2d(camToTarget.getTranslation().toTranslation2d(),
                                camToTarget.getRotation().toRotation2d())) // transform by cam to target
                        .transformBy(new Transform2d(new Translation2d(1, 0),
                                Rotation2d.fromDegrees(180))); // go to position
                System.out
                        .println("Camera to Target: " + new Transform2d(camToTarget.getTranslation().toTranslation2d(),
                                camToTarget.getRotation().toRotation2d()).toString());
                System.out.println("Pose to Target " + targetID + ": " + finalPose.toString());
                m_drivetrainSubsystem.getFollower().follow(new Trajectory(
                        new SimplePathBuilder(new Vector2(0, 0), Rotation2.fromDegrees(0)).lineTo(
                                new Vector2(finalPose.getX(), finalPose.getY()),
                                Rotation2.fromDegrees(finalPose.getRotation().getDegrees())).build(),
                        DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS,
                        AutonomousTrajectories.SAMPLE_DISTANCE));
            }
        }
    }

    @Override
    public void end(boolean interupted) {
        m_drivetrainSubsystem.getFollower().cancel();
    }

    @Override
    public boolean isFinished() {
        return m_drivetrainSubsystem.getFollower().getCurrentTrajectory().isEmpty();
    }
}
