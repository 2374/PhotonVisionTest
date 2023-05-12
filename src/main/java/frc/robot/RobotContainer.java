package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.util.AutonomousChooser;
import frc.robot.util.AutonomousTrajectories;

public class RobotContainer {
    private final ChassisSubsystem m_ChassisSubsystem;
    private final DrivetrainSubsystem m_DrivetrainSubsystem;

    private final AutonomousChooser autonomousChooser = new AutonomousChooser(
            new AutonomousTrajectories(DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS));

    private final XboxController m_driveController = new XboxController(Constants.CONTROLLER_USB_PORT_DRIVER);
    private final XboxController m_operatorController = new XboxController(Constants.CONTROLLER_USB_PORT_OPERATOR);

    private SlewRateLimiter xLimiter = new SlewRateLimiter(5);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(5);

    private boolean slow = false;
    private boolean roll = false;

    /**
     * The robot container. Need I say more?
     */
    public RobotContainer() {
        m_ChassisSubsystem = new ChassisSubsystem();
        m_DrivetrainSubsystem = new DrivetrainSubsystem(this);

        System.out.println("container created");

        // resetDrive();

        configureButtonBindings();
        configureShuffleBoard();
    }

    /**
     * Reset the default drive command
     */
    public void resetDrive() {
        m_DrivetrainSubsystem.setDefaultCommand(
                new DefaultDriveCommand(m_DrivetrainSubsystem, this::getForwardInput, this::getStrafeInput,
                        this::getRotationInput));
    }

    /**
     * Get the main controller
     * 
     * @return The main controller
     */
    public XboxController getMainController() {
        return m_driveController;
    }

    /**
     * Get the second controller
     * 
     * @return The second controller
     */
    public XboxController getSecondController() {
        return m_operatorController;
    }

    /**
     * Set up the Shuffleboard
     */
    public void configureShuffleBoard() {
        ShuffleboardTab tab = Shuffleboard.getTab(Constants.DRIVER_READOUT_TAB_NAME);
        // tab.add("setPointUp", new InstantCommand(() -> m_ArmSubsystem.setpointUP()));
        // tab.add("setPointBack", new InstantCommand(() ->
        // m_ArmSubsystem.setpointBACK()));
        // tab.add("setPointForward", new InstantCommand(() ->
        // m_ArmSubsystem.setpointFORWARD()));
        // tab.add("setPointDown", new InstantCommand(() ->
        // m_ArmSubsystem.setpointDOWN()));
        tab.add("Autonomous Mode", getAutonomousChooser().getModeChooser()).withSize(2, 1).withPosition(1, 0);
        // tab.add(m_drivetrainSubsystem.getField()).withSize(3, 2).withPosition(0, 1);
        tab.addBoolean("SLOW", () -> isSlow()).withPosition(1, 1);
        tab.addBoolean("ROLL", () -> isRoll()).withPosition(2, 1);
        // tab.addBoolean("Auto", () ->
        // m_drivetrainSubsystem.getFollower().getCurrentTrajectory().isPresent());
    }

    /**
     * Setup all of the button controls for the robot
     */
    public void configureButtonBindings() {
        // Drivetrain
        new Trigger(m_driveController::getBackButton)
                .onTrue(new InstantCommand(m_DrivetrainSubsystem::zeroRotation, m_DrivetrainSubsystem));
        new Trigger(m_driveController::getLeftBumper).debounce(0.1, DebounceType.kFalling)
                .onTrue(new InstantCommand(this::toggleSlow));
        new Trigger(m_driveController::getRightBumper).debounce(0.1, DebounceType.kFalling)
                .onTrue(new InstantCommand(this::toggleRoll));
    }

    /**
     * Adjusts the input to remove the tolerance while retaining a smooth line with
     * tolerance as 0 and 100 as 100
     * 
     * @param value     The value to adjust
     * @param tolerance The amount of inner area to remove
     * @return The adjusted value
     */
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
     * Accessor to the Autonomous Chooser
     * 
     * @return The Autonomous Chooser
     */
    public AutonomousChooser getAutonomousChooser() {
        return autonomousChooser;
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
