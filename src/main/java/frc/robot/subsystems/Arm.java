// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  private LoggedMechanismLigament2d m_ligament;
  /** Creates a new Arm. */
  public Arm(LoggedMechanismLigament2d ligament) {
    m_ligament = ligament;
    m_ligament.setColor(new Color8Bit(150, 150, 150));
    m_ligament.setLineWeight(3);
  }

  public Angle getAngle() {
    return Degrees.of(-90);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_ligament.setAngle(getAngle().in(Degrees));
  }
}
