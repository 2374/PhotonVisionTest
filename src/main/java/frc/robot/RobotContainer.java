package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.*;

public class RobotContainer {

    private final PhotonVisionSubsystem m_photonVisionSubsystem = new PhotonVisionSubsystem();
    private final ChassisSubsystem m_ChassisSubsystem;
    private final DrivetrainSubsystem m_DrivetrainSubsystem;

    private final XboxController m_driveController = new XboxController(Constants.CONTROLLER_PORT);

    private SlewRateLimiter xLimiter = new SlewRateLimiter(5);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(5);

    private boolean slow = false;
    private boolean roll = false;

    public RobotContainer() {
        m_ChassisSubsystem = new ChassisSubsystem();
        m_DrivetrainSubsystem = new DrivetrainSubsystem(this);

        configureButtonBindings();
    }

    public void resetDrive() {
        m_DrivetrainSubsystem.setDefaultCommand(
                new DefaultDriveCommand(m_DrivetrainSubsystem, this::getForwardInput, this::getStrafeInput,
                        this::getRotationInput));
    }

    public XboxController getController() {
        return m_driveController;
    }

    public void configureButtonBindings() {
        new Trigger(m_driveController::getBackButton)
                .onTrue(new InstantCommand(m_DrivetrainSubsystem::zeroRotation, m_DrivetrainSubsystem));
        new Trigger(m_driveController::getLeftBumper).debounce(0.1, DebounceType.kFalling)
                .onTrue(new InstantCommand(this::toggleSlow));
        new Trigger(m_driveController::getRightBumper).debounce(0.1, DebounceType.kFalling)
                .onTrue(new InstantCommand(this::toggleRoll));

        new Trigger(m_driveController::getAButton).onTrue(new InstantCommand(m_photonVisionSubsystem::update));
        new Trigger(m_driveController::getBButton).onTrue(new InstantCommand(m_photonVisionSubsystem::doStuff));
    }

    public static double deadband(double value, double tolerance) {
        if (Math.abs(value) < tolerance)
            return 0.0;

        return Math.copySign(value, (value - tolerance) / (1.0 - tolerance));
    }

    /**
     * Copy sign square
     * 
     * @param value Value to square
     * @return The copy sign square
     */
    public static double square(double value) {
        return Math.copySign(value * value, value);
    }

    /**
     * Get the adjusted Left Y axis of the main controller
     * 
     * @return The adjusted Left Y axis of the main controller
     */
    private double getForwardInput() {
        if (slow) {
            return -square(yLimiter.calculate(deadband(m_driveController.getLeftY(), 0.1))
                    * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
                    * DrivetrainSubsystem.SPEED_MULTIPLIER * 0.2);
        } else if (roll) {
            return -square(yLimiter.calculate(deadband(m_driveController.getLeftY(), 0.1))
                    * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
                    * DrivetrainSubsystem.SPEED_MULTIPLIER * .5);
        } else {
            return -square(yLimiter.calculate(deadband(m_driveController.getLeftY(), 0.1))
                    * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
                    * DrivetrainSubsystem.SPEED_MULTIPLIER);
        }
    }

    /**
     * Get the adjusted Left X axis of the main controller
     * 
     * @return The adjusted Left X axis of the main controller
     */
    private double getStrafeInput() {
        if (slow) {
            return -square(xLimiter.calculate(deadband(m_driveController.getLeftX(), 0.1))
                    * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
                    * DrivetrainSubsystem.SPEED_MULTIPLIER * 0.2);
        } else if (roll) {
            return -square(xLimiter.calculate(deadband(m_driveController.getLeftX(), 0.1))
                    * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
                    * DrivetrainSubsystem.SPEED_MULTIPLIER * 0.5);
        } else {
            return -square(xLimiter.calculate(deadband(m_driveController.getLeftX(), 0.1))
                    * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
                    * DrivetrainSubsystem.SPEED_MULTIPLIER);
        }
    }

    /**
     * Get the adjusted Right X axis of the main controller
     * 
     * @return The adjusted Right X axis of the main controller
     */
    private double getRotationInput() {
        if (slow) {
            return -square(deadband(m_driveController.getRightX(), 0.1))
                    * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * .2 * 0.33;
        } else {
            return -square(deadband(m_driveController.getRightX(), 0.1))
                    * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * .2;
        }
    }

    public boolean isSlow() {
        return slow;
    }

    public boolean isRoll() {
        return roll;
    }

    public void toggleSlow() {
        slow = !slow;
    }

    public void toggleRoll() {
        roll = !roll;
    }

    /**
     * Accessor to the Chassis Subsystem
     * 
     * @return The Chassis Subsystem
     */
    public ChassisSubsystem getChassisSubsystem() {
        return m_ChassisSubsystem;
    }

    /**
     * Accessor to the DriveTrain Subsystem
     * 
     * @return The DriveTrain Subsystem
     */
    public DrivetrainSubsystem getDrivetrain() {
        return m_DrivetrainSubsystem;
    }
}
