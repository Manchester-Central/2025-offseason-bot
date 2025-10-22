package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.FieldPoint;

public class autoaline extends Command{
  public FieldPoint trough;
  public Drive robot;
  public autoaline(
    FieldPoint feild, Drive drive
  ){
    trough = feild;
    robot = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    //
  }
}