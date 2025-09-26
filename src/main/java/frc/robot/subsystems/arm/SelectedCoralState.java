// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import frc.robot.Constants.ArmConstants.ArmPoses;
import frc.robot.subsystems.arm.Arm.ArmState;

/** Add your docs here. */
public enum SelectedCoralState {
  L1(ArmState.PREP_L1, ArmState.SCORE_L1, ArmPoses.HoldCoralL1),
  L2(ArmState.PREP_L2, ArmState.SCORE_L2, ArmPoses.HoldCoralL2),
  L3(ArmState.PREP_L3, ArmState.SCORE_L3, ArmPoses.HoldCoralL3),
  L4(ArmState.PREP_L4, ArmState.SCORE_L4, ArmPoses.HoldCoralL4);

  public final ArmState PrepState;
  public final ArmState ScoreState;
  public final ArmPose HoldPose;

  private SelectedCoralState(ArmState prepState, ArmState scoreState, ArmPose holdPose) {
    PrepState = prepState;
    ScoreState = scoreState;
    HoldPose = holdPose;
  }
}
