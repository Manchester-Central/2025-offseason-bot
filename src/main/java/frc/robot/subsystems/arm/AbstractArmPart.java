// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.Arm.ArmValues;
import java.util.function.Supplier;

/** A class that stores values/functions that any arm part should have. */
public abstract class AbstractArmPart extends SubsystemBase {
  private Supplier<ArmValues> m_armValuesSupplier;

  /**
   * The base constructor for any arm part.
   *
   * @param armValuesSupplier the supplier of arm values
   */
  protected AbstractArmPart(Supplier<ArmValues> armValuesSupplier) {
    m_armValuesSupplier = armValuesSupplier;
  }

  protected ArmValues getArmValues() {
    return m_armValuesSupplier.get();
  }
}
