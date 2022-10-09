package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.components.AutoDriveForwardForSeconds;
import frc.robot.subsystems.DrivetrainSubsystem;


public class AutoTaxi extends SequentialCommandGroup {
  public AutoTaxi(DrivetrainSubsystem drivetrain) {
      addCommands(
        new AutoDriveForwardForSeconds(drivetrain, 2)
      );
  }
}
