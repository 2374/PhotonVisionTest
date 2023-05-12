package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ChassisSubsystem extends SubsystemBase {

    private String serialNumber = "unknown";

    private static ChassisSubsystem instance;

    /**
     * Handles robot wide and generic systems
     */
    public ChassisSubsystem() {
        instance = this;
        serialNumber = RobotController.getSerialNumber();
        System.out.println("SERIALNUMBER=" + serialNumber);
        // SmartDashboard.putData(camera); fix this
    }

    /**
     * Gets the ChassisSubsystem instance. Makes a new one if not present.
     * 
     * @return The ChassisSubsystem
     */
    public static ChassisSubsystem getChassisInstance() {
        if (instance == null) {
            instance = new ChassisSubsystem();
        }
        return instance;
    }

    /**
     * Is this swervee?
     * 
     * @return Is this swervee?
     */
    public Boolean isTestRobot() {
        Boolean result = serialNumber.equalsIgnoreCase(Constants.TEST_ROBORIO_SERIAL_NUMBER);
        System.out.println("RIOTEST=" + result);
        return result;
    }

}
