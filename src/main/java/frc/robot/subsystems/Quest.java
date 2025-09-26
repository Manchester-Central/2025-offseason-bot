// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.QuestNav;

public class Quest extends SubsystemBase {
  QuestNav m_questNav = new QuestNav();
  /** Creates a new Quest. */
  public Quest() {}

  @Override
  public void periodic() {
    m_questNav.commandPeriodic();
    // This method will be called once per scheduler run
  }
}

