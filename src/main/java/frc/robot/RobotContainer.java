// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.chaos131.gamepads.Gamepad;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ArmConstants.ArmPoses;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Quest;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Quest m_quest;
  private final Drive m_swerveDrive;
  private final Gripper m_gripper;
  private final Arm m_arm;

  // Controller
  private final Gamepad m_driver = new Gamepad(0);
  private final Gamepad m_operator = new Gamepad(1);

  // Mechanism2d Simulation Support
  @AutoLogOutput(key = "Mech2d")
  private final LoggedMechanism2d m_mechanism2d = new LoggedMechanism2d(0, 0);
  private final LoggedMechanismRoot2d m_mechanismRoot2d = m_mechanism2d.getRoot("ArmRoot", 0, 0);
  private final LoggedMechanismLigament2d m_originToPivot =
      m_mechanismRoot2d.append(new LoggedMechanismLigament2d("Supports", 0.4, 90));
  //                                                                          meters,     degrees
  private final LoggedMechanismLigament2d m_armLigament =
      m_originToPivot.append(new LoggedMechanismLigament2d("Arm", 0.6, -90));
  private final LoggedMechanismLigament2d m_gripperLigament = 
      m_armLigament.append(new LoggedMechanismLigament2d("Gripper", 0.2, 0));

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> m_autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        m_swerveDrive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        m_swerveDrive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        break;

      default:
        // Replayed robot, disable IO implementations
        m_swerveDrive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }
    m_quest = new Quest(m_swerveDrive);
    // Set up auto routines
    m_autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    m_autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(m_swerveDrive));
    m_autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(m_swerveDrive));
    m_autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        m_swerveDrive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        m_swerveDrive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    m_autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", m_swerveDrive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    m_autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", m_swerveDrive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    m_arm = new Arm(m_armLigament);
    m_gripper = new Gripper(m_gripperLigament);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    m_swerveDrive.setDefaultCommand(
        DriveCommands.joystickDrive(
            m_swerveDrive,
            () -> m_driver.getLeftY(),
            () -> -m_driver.getLeftX(),
            () -> -m_driver.getRightX()));

    // Lock to 0° when A button is held
    m_driver
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                m_swerveDrive,
                () -> m_driver.getLeftY(),
                () -> -m_driver.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    m_driver.x().onTrue(Commands.runOnce(m_swerveDrive::stopWithX, m_swerveDrive));

    // Reset gyro to 0° when B button is pressed
    m_driver
        .povUp()
        .onTrue(
            Commands.runOnce(
                    () ->
                        m_swerveDrive.setPose(
                            new Pose2d(m_swerveDrive.getPose().getTranslation(), new Rotation2d())),
                    m_swerveDrive)
                .ignoringDisable(true));

    m_arm.setDefaultCommand(new InstantCommand(() -> m_arm.setSpeed(m_operator.getLeftY() * 0.5), m_arm));

    m_driver.leftTrigger().whileTrue(new InstantCommand(() -> m_arm.setTargetAngle((Angle)ArmPoses.HPIntakePose.get()), m_arm)
      .alongWith(new InstantCommand(() -> m_gripper.setGripSpeed(-0.6), m_gripper)));

    m_driver.rightBumper().whileTrue(new InstantCommand(() -> m_arm.setTargetAngle((Angle)ArmPoses.ScoreLowPose.get()), m_arm));

    m_driver.rightTrigger().whileTrue(new InstantCommand(() -> m_gripper.setGripSpeed(0.6), m_gripper));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.get();
  }

  public void logMech3d() {
    // Robot specific parts
    Pose3d[] parts = {
      new Pose3d(),
      new Pose3d(new Translation3d(0, 0, m_originToPivot.getLength()),
                 new Rotation3d(Degrees.of(0), m_arm.getCurrentAngle().times(-1), Degrees.of(0))),
    };
    // Note, the arm's angle is multiplied by -1 because RollPitchYaw uses slightly different directions
    // than many think are intuitive, +pitch is actually down because of the left facing +y direction
    Logger.recordOutput("Mech3d", parts);

    // Coral held by the robot (shown as game piece), must be in field relative coordinate frame
    Pose3d[] held_coral_position = {};
    if (m_gripper.hasCoral()) {
      held_coral_position = new Pose3d[]{
        // Insert Mech2d forward kinematics to get orientation, for now plot it at the origin
        new Pose3d(new Translation3d(0,0,2.0), new Rotation3d())
      };
    } 
    Logger.recordOutput("heldCoralPosition", held_coral_position);
  }
}
