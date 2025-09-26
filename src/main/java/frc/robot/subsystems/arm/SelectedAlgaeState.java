// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import frc.robot.subsystems.arm.Arm.ArmState;

/** Add your docs here. */
public enum SelectedAlgaeState {
  BARGE(ArmState.PREP_BARGE),
  PROCESSOR(ArmState.PREP_PROCESSOR);

  public final ArmState State;

  private SelectedAlgaeState(ArmState state) {
    State = state;
  }
}
