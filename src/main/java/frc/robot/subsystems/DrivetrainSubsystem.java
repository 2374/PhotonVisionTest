package frc.robot.subsystems;

import java.util.Map;
import com.ctre.phoenix.sensors.Pigeon2;
import com.swervedrivespecialties.swervelib.Mk3ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
// import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
// import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.util.PhotonCameraWrapper;
import frc.robot.util.Utilities;
import frc.common.control.*;
import frc.common.math.Vector2;
import frc.common.util.DrivetrainFeedforwardConstants;
import frc.common.util.HolonomicDriveSignal;
import frc.common.util.HolonomicFeedforward;

import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {
    public static double SPEED_MULTIPLIER = .65;
    public static final double MAX_VOLTAGE = 12.0;
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0
            * SdsModuleConfigurations.MK3_FAST.getDriveReduction() * SdsModuleConfigurations.MK3_FAST.getWheelDiameter()
            * Math.PI;
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND
            / Math.hypot(DRIVETRAIN_LENGTH_METERS / 2.0, DRIVETRAIN_WIDTH_METERS / 2.0);

    public static final DrivetrainFeedforwardConstants FEEDFORWARD_CONSTANTS = new DrivetrainFeedforwardConstants(0.891,
            0.15, 0.13592);

    public static final double DRIVETRAIN_CURRENT_LIMIT = 50.0;

    public static final TrajectoryConstraint[] TRAJECTORY_CONSTRAINTS = {
            new FeedforwardConstraint(1.5, FEEDFORWARD_CONSTANTS.getVelocityConstant(),
                    FEEDFORWARD_CONSTANTS.getAccelerationConstant(), false),
            new MaxAccelerationConstraint(3.0), new CentripetalAccelerationConstraint(1.0) };

    private final HolonomicMotionProfiledTrajectoryFollower follower = new HolonomicMotionProfiledTrajectoryFollower(
            new PidConstants(1, 0.02, .06), new PidConstants(1, 0.02, .06),
            new HolonomicFeedforward(FEEDFORWARD_CONSTANTS));

    private final SwerveDriveKinematics kinematics;

    private final SwerveDrivePoseEstimator estimator;

    private final Pigeon2 pigeon = new Pigeon2(DRIVETRAIN_PIGEON_CAN_ID, Constants.CAN_BUS_NAME_DRIVETRAIN);

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private ChassisSpeeds currentVelocity = new ChassisSpeeds();
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    private final GenericEntry motorOutputPercentageLimiterEntry;

    private double motorOutputLimiter;

    private final Field2d m_field = new Field2d();

    private final PhotonCameraWrapper pcw;

    public DrivetrainSubsystem(RobotContainer container) {
        pcw = new PhotonCameraWrapper();
        pigeon.configMountPose(180, 0, 0);
        Shuffleboard.getTab(Constants.DRIVER_READOUT_TAB_NAME).addNumber("yaw", () -> pigeon.getYaw());
        // Mk4ModuleConfiguration mk4ModuleConfiguration = new Mk4ModuleConfiguration();
        // mk4ModuleConfiguration.setDriveCurrentLimit(DRIVETRAIN_CURRENT_LIMIT);
        Mk3ModuleConfiguration mk3ModuleConfiguration = new Mk3ModuleConfiguration();
        mk3ModuleConfiguration.setDriveCurrentLimit(DRIVETRAIN_CURRENT_LIMIT);
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        kinematics = new SwerveDriveKinematics(
                // Front left
                new Translation2d(
                        (container.getChassisSubsystem().isTestRobot() ? Constants.DRIVETRAIN_LENGTH_METERS_TEST
                                : Constants.DRIVETRAIN_LENGTH_METERS) / 2.0,
                        (container.getChassisSubsystem().isTestRobot() ? Constants.DRIVETRAIN_WIDTH_METERS_TEST
                                : Constants.DRIVETRAIN_WIDTH_METERS) / 2.0),
                // Front right
                new Translation2d(
                        (container.getChassisSubsystem().isTestRobot() ? Constants.DRIVETRAIN_LENGTH_METERS_TEST
                                : Constants.DRIVETRAIN_LENGTH_METERS) / 2.0,
                        -(container.getChassisSubsystem().isTestRobot() ? Constants.DRIVETRAIN_WIDTH_METERS_TEST
                                : Constants.DRIVETRAIN_WIDTH_METERS) / 2.0),
                // Back left
                new Translation2d(
                        -(container.getChassisSubsystem().isTestRobot() ? Constants.DRIVETRAIN_LENGTH_METERS_TEST
                                : Constants.DRIVETRAIN_LENGTH_METERS) / 2.0,
                        (container.getChassisSubsystem().isTestRobot() ? Constants.DRIVETRAIN_WIDTH_METERS_TEST
                                : Constants.DRIVETRAIN_WIDTH_METERS) / 2.0),
                // Back right
                new Translation2d(
                        -(container.getChassisSubsystem().isTestRobot() ? Constants.DRIVETRAIN_LENGTH_METERS_TEST
                                : Constants.DRIVETRAIN_LENGTH_METERS) / 2.0,
                        -(container.getChassisSubsystem().isTestRobot() ? Constants.DRIVETRAIN_WIDTH_METERS_TEST
                                : Constants.DRIVETRAIN_WIDTH_METERS) / 2.0));
        // frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
        // tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2,
        // 4).withPosition(0, 0),
        // mk4ModuleConfiguration, Mk4iSwerveModuleHelper.GearRatio.L3,
        // FRONT_LEFT_MODULE_DRIVE_MOTOR,
        // FRONT_LEFT_MODULE_STEER_MOTOR, FRONT_LEFT_MODULE_STEER_ENCODER,
        // FRONT_LEFT_MODULE_STEER_OFFSET);
        // frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
        // tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2,
        // 4).withPosition(2, 0),
        // mk4ModuleConfiguration, Mk4iSwerveModuleHelper.GearRatio.L3,
        // FRONT_RIGHT_MODULE_DRIVE_MOTOR,
        // FRONT_RIGHT_MODULE_STEER_MOTOR, FRONT_RIGHT_MODULE_STEER_ENCODER,
        // FRONT_RIGHT_MODULE_STEER_OFFSET);
        // backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
        // tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2,
        // 4).withPosition(4, 0),
        // mk4ModuleConfiguration, Mk4iSwerveModuleHelper.GearRatio.L3,
        // BACK_LEFT_MODULE_DRIVE_MOTOR,
        // BACK_LEFT_MODULE_STEER_MOTOR, BACK_LEFT_MODULE_STEER_ENCODER,
        // BACK_LEFT_MODULE_STEER_OFFSET);
        // backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
        // tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2,
        // 4).withPosition(6, 0),
        // mk4ModuleConfiguration, Mk4iSwerveModuleHelper.GearRatio.L3,
        // BACK_RIGHT_MODULE_DRIVE_MOTOR,
        // BACK_RIGHT_MODULE_STEER_MOTOR, BACK_RIGHT_MODULE_STEER_ENCODER,
        // BACK_RIGHT_MODULE_STEER_OFFSET);
        frontLeftModule = Mk3SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0),
                mk3ModuleConfiguration, Mk3SwerveModuleHelper.GearRatio.FAST, FRONT_LEFT_MODULE_DRIVE_MOTOR_CAN_ID,
                FRONT_LEFT_MODULE_STEER_MOTOR_CAN_ID, FRONT_LEFT_MODULE_STEER_ENCODER_CAN_ID,
                container.getChassisSubsystem().isTestRobot() ? FRONT_LEFT_MODULE_STEER_OFFSET_TEST
                        : FRONT_LEFT_MODULE_STEER_OFFSET);
        frontRightModule = Mk3SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0),
                mk3ModuleConfiguration, Mk3SwerveModuleHelper.GearRatio.FAST, FRONT_RIGHT_MODULE_DRIVE_MOTOR_CAN_ID,
                FRONT_RIGHT_MODULE_STEER_MOTOR_CAN_ID, FRONT_RIGHT_MODULE_STEER_ENCODER_CAN_ID,
                container.getChassisSubsystem().isTestRobot() ? FRONT_RIGHT_MODULE_STEER_OFFSET_TEST
                        : FRONT_RIGHT_MODULE_STEER_OFFSET);
        backLeftModule = Mk3SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0),
                mk3ModuleConfiguration, Mk3SwerveModuleHelper.GearRatio.FAST, BACK_LEFT_MODULE_DRIVE_MOTOR_CAN_ID,
                BACK_LEFT_MODULE_STEER_MOTOR_CAN_ID, BACK_LEFT_MODULE_STEER_ENCODER_CAN_ID,
                container.getChassisSubsystem().isTestRobot() ? BACK_LEFT_MODULE_STEER_OFFSET_TEST
                        : BACK_LEFT_MODULE_STEER_OFFSET);
        backRightModule = Mk3SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0),
                mk3ModuleConfiguration, Mk3SwerveModuleHelper.GearRatio.FAST, BACK_RIGHT_MODULE_DRIVE_MOTOR_CAN_ID,
                BACK_RIGHT_MODULE_STEER_MOTOR_CAN_ID, BACK_RIGHT_MODULE_STEER_ENCODER_CAN_ID,
                container.getChassisSubsystem().isTestRobot() ? BACK_RIGHT_MODULE_STEER_OFFSET_TEST
                        : BACK_RIGHT_MODULE_STEER_OFFSET);

        estimator = new SwerveDrivePoseEstimator(kinematics, getGyroscopeRotation(), getSwerveModulePositions(),
                new Pose2d()); // Vision (x, y, rotation) std-devs
        // estimator = new SwerveDrivePoseEstimator(getGyroscopeRotation(), new
        // Pose2d(), kinematics,
        // VecBuilder.fill(0.02, 0.02, 0.01), // estimator values (x, y, rotation)
        // std-devs
        // VecBuilder.fill(0.01), // Gyroscope rotation std-dev
        // VecBuilder.fill(0.1, 0.1, 0.01)); // Vision (x, y, rotation) std-devs
        motorOutputPercentageLimiterEntry = tab.add("Motor Percentage", 100.0).withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0.0, "max", 100.0, "Block increment", 10.0)).withPosition(0, 3)
                .getEntry();

        tab.addNumber("Odometry X", () -> Units.metersToFeet(getPose().getX()));
        tab.addNumber("Odometry Y", () -> Units.metersToFeet(getPose().getY()));
        tab.addNumber("Odometry Angle", () -> getPose().getRotation().getDegrees());
        tab.addNumber("Velocity X", () -> Units.metersToFeet(getCurrentVelocity().vxMetersPerSecond));
        tab.addNumber("Trajectory Position X", () -> {
            var lastState = follower.getLastState();
            if (lastState == null)
                return 0;

            return Units.metersToFeet(lastState.getPathState().getPosition().x);
        });

        tab.addNumber("Trajectory Velocity X", () -> {
            var lastState = follower.getLastState();
            if (lastState == null)
                return 0;

            return Units.metersToFeet(lastState.getVelocity());
        });
        tab.addNumber("Gyroscope Angle", () -> getGyroscopeRotation().getDegrees());
    }

    public Field2d getField() {
        return m_field;
    }

    private Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(pigeon.getYaw());
    }

    private SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] smp = new SwerveModulePosition[] {
                frontLeftModule.getPosition(), frontRightModule.getPosition(),
                backLeftModule.getPosition(), backRightModule.getPosition()
        };
        return smp;
    }

    /**
     * Resets the rotation of the drivetrain to zero.
     */
    public void zeroRotation() {
        System.out.println("reset");
        estimator.resetPosition(getGyroscopeRotation(), getSwerveModulePositions(),
                new Pose2d(getPose().getX(), getPose().getY(), new Rotation2d()));
    }

    /**
     * Returns the position of the robot
     */
    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

    public double getMotorOutputLimiter() {
        return motorOutputLimiter;
    }

    public HolonomicMotionProfiledTrajectoryFollower getFollower() {
        return follower;
    }

    public ChassisSpeeds getCurrentVelocity() {
        return currentVelocity;
    }

    /**
     * Sets the position of the robot to the position passed in with the current
     * gyroscope rotation.
     */
    public void setPose(Pose2d pose) {
        estimator.resetPosition(getGyroscopeRotation(), getSwerveModulePositions(), pose);
    }

    /**
     * Sets the desired chassis speed of the drivetrain.
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
        // System.out.println("Chassis speed"+chassisSpeeds.vxMetersPerSecond);
    }

    public void periodic() {
        SwerveModuleState currentFrontLeftModuleState = new SwerveModuleState(frontLeftModule.getDriveVelocity(),
                new Rotation2d(frontLeftModule.getSteerAngle()));
        SwerveModuleState currentFrontRightModuleState = new SwerveModuleState(frontRightModule.getDriveVelocity(),
                new Rotation2d(frontRightModule.getSteerAngle()));
        SwerveModuleState currentBackLeftModuleState = new SwerveModuleState(backLeftModule.getDriveVelocity(),
                new Rotation2d(backLeftModule.getSteerAngle()));
        SwerveModuleState currentBackRightModuleState = new SwerveModuleState(backRightModule.getDriveVelocity(),
                new Rotation2d(backRightModule.getSteerAngle()));

        currentVelocity = kinematics.toChassisSpeeds(currentFrontLeftModuleState, currentFrontRightModuleState,
                currentBackLeftModuleState, currentBackRightModuleState);

        estimator.update(getGyroscopeRotation(), getSwerveModulePositions());

        // Vision stuff // No Field relative until need to align // do not uncomment
        // Optional<EstimatedRobotPose> result =
        // pcw.getEstimatedGlobalPose(estimator.getEstimatedPosition());
        // if (result.isPresent()) {
        // EstimatedRobotPose camPose = result.get();
        // estimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), // REMOVE
        // THE ROTATION PART
        // camPose.timestampSeconds);
        // m_field.getObject("Cam Est Pos").setPose(camPose.estimatedPose.toPose2d());
        // } else {
        // m_field.getObject("Cam Est Pos").setPose(new Pose2d(-100, -100, new
        // Rotation2d()));
        // }
        // m_field.getObject("Actual Pos").setPose(getPose());

        m_field.setRobotPose(estimator.getEstimatedPosition());

        var driveSignalOpt = follower.update(Utilities.poseToRigidTransform(getPose()),
                new Vector2(currentVelocity.vxMetersPerSecond, currentVelocity.vyMetersPerSecond),
                currentVelocity.omegaRadiansPerSecond, Timer.getFPGATimestamp(), Robot.kDefaultPeriod);

        if (driveSignalOpt.isPresent()) {
            HolonomicDriveSignal driveSignal = driveSignalOpt.get();
            // System.out.println(driveSignal.getTranslation().x);
            if (driveSignalOpt.get().isFieldOriented()) {
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(driveSignal.getTranslation().x,
                        driveSignal.getTranslation().y, driveSignal.getRotation(),
                        getPose().getRotation());
            } else {
                chassisSpeeds = new ChassisSpeeds(driveSignal.getTranslation().x,
                        driveSignal.getTranslation().y,
                        driveSignal.getRotation());
            }
        }

        motorOutputLimiter = motorOutputPercentageLimiterEntry.getDouble(0.0) / 100;
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
        // if (false) {
        // states = new SwerveModuleState[] {
        // new SwerveModuleState(0, -45)
        // }
        // }

        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
        frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[0].angle.getRadians());
        frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[1].angle.getRadians());
        backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[2].angle.getRadians());
        backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[3].angle.getRadians());
    }

    public PhotonCameraWrapper getPhotonCameraWrapper() {
        return pcw;
    }

    // public void autoBalenceTick() {
    // double theta;
    // double pitch = pigeon.getPitch();
    // double roll = pigeon.getRoll();
    // if (Math.abs(roll) <= 2)
    // if (Math.abs(pitch) <= 2)
    // theta = Double.NaN;
    // else
    // theta = 0;
    // else if (Math.abs(pitch) <= 2)
    // theta = 90;
    // else
    // theta = Math.toDegrees(Math
    // .atan(Math.sin(Math.toRadians(roll)) / Math.sin(Math.toRadians(pitch)))) +
    // 90;
    // System.out.println("Pitch - " + pitch);
    // System.out.println("Roll - " + roll);
    // System.out.println("Theta - " + theta);
    // SmartDashboard.putNumber("Theta", theta);
    // if (theta != Double.NaN)
    // drive(new ChassisSpeeds(Math.sin(Math.toRadians(theta)) * SPEED_MULTIPLIER,
    // Math.cos(Math.toRadians(theta)) * SPEED_MULTIPLIER, 0));
    // }

    public void autoBalenceTick() {
        double pitch = pigeon.getPitch();
        if (pitch > 2.5) {
            drive(new ChassisSpeeds(Math.min(0.4, pitch / 30), 0, 0));
        } else if (pitch < -2.5) {
            drive(new ChassisSpeeds(Math.max(-0.4, pitch / 30), 0, 0));
        } else {
            drive(new ChassisSpeeds(0, 0, 0));
        }
    }

    public void printAngles() {
        System.out.println("Pitch - " + pigeon.getPitch());
        System.out.println("Roll - " + pigeon.getRoll());
    }

    public double getPitch() {
        return pigeon.getPitch();
    }

    public double getYaw() {
        return pigeon.getYaw();
    }
}