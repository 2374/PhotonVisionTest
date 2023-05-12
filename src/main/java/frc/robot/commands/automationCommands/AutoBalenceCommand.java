package frc.robot.commands.automationCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoBalenceCommand extends CommandBase {

    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private int count = 0;

    public AutoBalenceCommand(DrivetrainSubsystem drivetrainSubsystem) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // if (m_drivetrainSubsystem.autoBalenceTick()) {
        // count++;
        // }
        // if (count > 100) {
        // end(false);
        // }
    }

    @Override
    public void end(boolean interupted) {
        if (!interupted) {
            System.out.println("completed");
        } else {
            System.out.println("timed");
        }
    }
}
