package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import frc.common.math.RigidTransform2;
import frc.common.math.Rotation2;
import frc.common.math.Vector2;

public class Utilities {
    public static RigidTransform2 poseToRigidTransform(Pose2d pose) {
        return new RigidTransform2(new Vector2(pose.getX(), pose.getY()),
                Rotation2.fromDegrees(pose.getRotation().getDegrees()));
    }
}
