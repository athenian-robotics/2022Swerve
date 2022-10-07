package frc.robot.commands.auto.components;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoDriveForwardForSeconds extends CommandBase {
    DrivetrainSubsystem drivetrain;
    Timer driveTimer;
    double secondsToDrive;

    public AutoDriveForwardForSeconds(DrivetrainSubsystem drivetrain, double secondsToDrive) {
        this.drivetrain = drivetrain;
        this.secondsToDrive = secondsToDrive;
        driveTimer = new Timer();
    }

    public void initialize() {
        driveTimer.reset();
        driveTimer.start();
    }

    public void execute() {
        drivetrain.tankDrive(0.4, 0.4);
    }

    public boolean isFinished() {
        return driveTimer.advanceIfElapsed(secondsToDrive);
    }

    public void end(boolean interrupted) {
        drivetrain.tankDrive(0, 0);
    }

}