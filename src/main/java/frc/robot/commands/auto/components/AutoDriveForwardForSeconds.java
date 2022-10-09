package frc.robot.commands.auto.components;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import static frc.robot.subsystems.DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;

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
        drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
            0.0,
            1*MAX_VELOCITY_METERS_PER_SECOND*.1,
            0,
            new Rotation2d()
      ));
    }

    public boolean isFinished() {
        return driveTimer.advanceIfElapsed(secondsToDrive);
    }

    public void end(boolean interrupted) {
        drivetrain.tankDrive(0, 0);
    }

}