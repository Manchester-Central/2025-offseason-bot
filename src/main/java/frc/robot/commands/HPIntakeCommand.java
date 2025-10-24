// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants.ArmPoses;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HPIntakeCommand extends Command {
  private Arm m_arm;
  private Gripper m_gripper;

  /** Creates a new ScorePrep. */
  public HPIntakeCommand(Arm arm, Gripper gripper) {
    m_arm = arm;
    m_gripper = gripper;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_arm, m_gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.setTargetAngle(ArmPoses.HPIntakePose.get());
    m_gripper.setGripSpeed(0.6);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_gripper.hasCoral();
  }
}
