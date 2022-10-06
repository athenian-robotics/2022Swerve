package frc.robot.commands.auto.components;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoDriveForwardForSeconds extends Command {
    DrivetrainSubsystem drivetrain;
    Timer driveTimer = new Timer();
    double secondsTodrive;

    public AutoDriveForwardForSeconds(DrivetrainSubsystem drivetrain, double secondsToDrive) {
        this.drivetrain = drivetrain;
        this.secondsTodrive = secondsToDrive;

    }

    public void initialize() {
        driveTimer.reset();
        driveTimer.start();
    }

    public void execute() {
        drivetrain.tankDrive(0.4, 0.4);
    }

    public boolean isFinished() {
        return driveTimer.hasElapsed(secondsTodrive);
    }

    public void end(boolean interrupted) {
        drivetrain.tankDrive(0, 0);
    }

}