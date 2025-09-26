// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

import com.chaos131.util.DashboardNumber;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import frc.robot.Constants.CanIdentifiers;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Robot;
import frc.robot.subsystems.arm.Arm.ArmValues;
import frc.robot.utils.ChaosTalonFx;
import frc.robot.utils.ChaosTalonFxTuner;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Climber extends AbstractArmPart {
  private static boolean m_hasCageGripped = false;

  public static boolean hasCageGrippedSim = false;

  private ChaosTalonFx m_climbMotor = new ChaosTalonFx(CanIdentifiers.ClimberMotorCANID, CanIdentifiers.RioCANBus);

  private Debouncer m_cageSensorDebouncer = new Debouncer(ClimberConstants.CageDropDebounceSeconds, DebounceType.kBoth);

  private ChaosTalonFxTuner m_cageTuner = new ChaosTalonFxTuner("Climber", m_climbMotor);

  // Current limits
  private DashboardNumber m_cageSupplyCurrentLimit = m_cageTuner.tunable(
      "SupplyCurrentLimit", ClimberConstants.ClimbSupplyCurrentLimit.in(Amps), (config, newValue) -> config.CurrentLimits.SupplyCurrentLimit = newValue);
  private DashboardNumber m_cageStatorCurrentLimit = m_cageTuner.tunable(
      "StatorCurrentLimit", ClimberConstants.ClimbStatorCurrentLimit.in(Amps), (config, newValue) -> config.CurrentLimits.StatorCurrentLimit = newValue);
      

  /**
   * Creates a new Climber.
   *
   * @param armValuesSupplier the supplier of arm values
   */
  public Climber(Supplier<ArmValues> armValuesSupplier) {
    super(armValuesSupplier);
    m_climbMotor.Configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    m_climbMotor.Configuration.CurrentLimits.StatorCurrentLimit = m_cageStatorCurrentLimit.get();
    m_climbMotor.Configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_climbMotor.Configuration.CurrentLimits.SupplyCurrentLimit = m_cageSupplyCurrentLimit.get();
    m_climbMotor.Configuration.CurrentLimits.SupplyCurrentLowerLimit = ClimberConstants.ClimbSupplyCurrentLowerLimit.in(Amps);
    m_climbMotor.Configuration.CurrentLimits.SupplyCurrentLowerTime = ClimberConstants.ClimbSupplyCurrentLowerTime.in(Seconds);
    m_climbMotor.Configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_climbMotor.applyConfig();
  }

  /**
   * Sets the speed [-1.0, 1.0] of the climber.
   */
  public void setClimbSpeed(double newSpeed) {
    m_climbMotor.set(newSpeed);
  }

  /**
   * returns the current speed of the climber.
   */
  public double getClimbSpeed() {
    return m_climbMotor.get();
  }

  /**
   * Checks if we have the cage gripped.
   */
  public boolean hasCage() {
    return m_hasCageGripped;
  }

  @Override
  public void periodic() {
    super.periodic();
    boolean cageCurrentLimitReached = m_climbMotor.getStatorCurrent().getValue().gt(Amps.of(m_cageStatorCurrentLimit.get() - 0.1));
    m_hasCageGripped = m_cageSensorDebouncer.calculate(Robot.isSimulation() ? hasCageGrippedSim : cageCurrentLimitReached);
    Logger.recordOutput("Climber/HasCage", hasCage());
  }
}
