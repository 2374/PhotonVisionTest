package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionSubsystem extends SubsystemBase {
    PhotonCamera camera = new PhotonCamera("photonvision");
    PhotonTrackedTarget target = null;

    public PhotonVisionSubsystem() {

    }

    public void update() {
        var result = camera.getLatestResult();
        if (result.hasTargets()) {
            target = result.getBestTarget();
            System.out.print("GOOD ");
        }
        System.out.println("UPDATE");
    }

    public void doStuff() {
        if (target != null) {
            System.out.println("Pitch - " + target.getPitch());
            System.out.println("Skew - " + target.getSkew());
            System.out.println("Yaw - " + target.getYaw());
        } else {
            System.out.println("HELP");
        }
    }
}
