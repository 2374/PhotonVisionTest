package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.automationCommands.AlignWithAprilTagCommand;
import frc.robot.commands.automationCommands.AutoBalenceCommand;

public class Robot extends TimedRobot {
    private final RobotContainer m_robotContainer = new RobotContainer();

    private Command m_autonomousCommand;

    // @SuppressWarnings("unused")
    // private final CharacterizeDrivetrainCommand characterizeCommand = new
    // CharacterizeDrivetrainCommand(
    // m_robotContainer.getDrivetrain());

    @Override
    public void robotInit() {
        // drivetrain
        SmartDashboard.putData("Auto Balence", new AutoBalenceCommand(m_robotContainer.getDrivetrain()).withTimeout(5));
        SmartDashboard.putNumber("Apriltag Number", 1);
        SmartDashboard.putData("Align With Apriltag", new AlignWithAprilTagCommand(m_robotContainer.getDrivetrain(),
                (int) SmartDashboard.getNumber("Apriltag Number", 1)));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        // m_robotContainer.getChassisSubsystem().setWantNothing();
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void teleopInit() {
        // if (!robotContainer.getClimber().isClimberZeroed()) {
        // new ZeroClimberCommand(robotContainer.getClimber()).schedule();
        // }
        // if (!robotContainer.getShooter().isHoodZeroed()) {
        // new ZeroHoodCommand(robotContainer.getShooter(), true).schedule();
        // }
        m_robotContainer.resetDrive();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void testInit() {
        // new InstantCommand(robotContainer.getShooter()::disableFlywheel);
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void autonomousInit() {
        if (m_robotContainer.getDrivetrain().getDefaultCommand() != null)
            m_robotContainer.getDrivetrain().getDefaultCommand().cancel();
        m_autonomousCommand = m_robotContainer.getAutonomousChooser().getCommand(m_robotContainer);
        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousExit() {

    }
}
