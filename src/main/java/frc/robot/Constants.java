package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final String CANIVORE_CAN_BUS_NAME = "FastFD";
    public static final String RIO_CAN_BUS_NAME = "rio";
    public static final String CAN_BUS_NAME_DRIVETRAIN = CANIVORE_CAN_BUS_NAME;

    public static final int CONTROLLER_PORT = 0;

    public static final int ARM_MOTOR_PORT = 8;

    public static final int FEEDER_SENSOR_FULL_PORT = 0; // 1;
    public static final int FEEDER_SENSOR_ENTRY_PORT = 0;

    // DRIVETRAIN Subsystem
    public static final double DRIVETRAIN_LENGTH_METERS = Units.inchesToMeters(20.5);
    public static final double DRIVETRAIN_WIDTH_METERS = Units.inchesToMeters(20.5);
    public static final double DRIVETRAIN_LENGTH_METERS_TEST = Units.inchesToMeters(19);
    public static final double DRIVETRAIN_WIDTH_METERS_TEST = Units.inchesToMeters(20.5);
    public static final int DRIVETRAIN_PIGEON_CAN_ID = 29; // the CAN ID for the FASTFD CAN Bus
    // Front Left Swerve Module
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR_CAN_ID = 1; // the CAN ID for the FASTFD CAN Bus
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR_CAN_ID = 11; // the CAN ID for the FASTFD CAN Bus
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER_CAN_ID = 21; // the CAN ID for the FASTFD CAN Bus
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET_TEST = -Math.toRadians(190.27); // Swervee Module Offset
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(182.19 + 180); // Comp Module Offset
    // Front Right Swerve Module
    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR_CAN_ID = 2; // the CAN ID for the FASTFD CAN Bus
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR_CAN_ID = 12; // the CAN ID for the FASTFD CAN Bus
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER_CAN_ID = 22; // the CAN ID for the FASTFD CAN Bus
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET_TEST = -Math.toRadians(261.88); // Swervee Module Offset
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(214.1 + 180); // Comp Module Offset
    // Back Left Swerve Module
    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR_CAN_ID = 3; // the CAN ID for the FASTFD CAN Bus
    public static final int BACK_LEFT_MODULE_STEER_MOTOR_CAN_ID = 13; // the CAN ID for the FASTFD CAN Bus
    public static final int BACK_LEFT_MODULE_STEER_ENCODER_CAN_ID = 23; // the CAN ID for the FASTFD CAN Bus
    public static final double BACK_LEFT_MODULE_STEER_OFFSET_TEST = -Math.toRadians(126.02); // Swervee Module Offset
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(57.83); // Comp Module Offset
    // Back Right Swerve Module
    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR_CAN_ID = 4; // the CAN ID for the FASTFD CAN Bus
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR_CAN_ID = 14; // the CAN ID for the FASTFD CAN Bus
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER_CAN_ID = 24; // the CAN ID for the FASTFD CAN Bus
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET_TEST = -Math.toRadians(263.67); // Swervee Module Offset
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(154.41 + 180); // Comp Module Offset

    public static final String DRIVER_READOUT_TAB_NAME = "Driver Readout";

    public static final String TEST_ROBORIO_SERIAL_NUMBER = "0316b2d6"; // serial number of Swervee roborio

    public static final String CAMERA_NAME = "photonvision";
    public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(new Translation3d(.3, 0, 0.2),
            new Rotation3d(0, 0, 0));
}
