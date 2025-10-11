package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.utils.FieldMirroringUtils;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

public class AdditionalSimRobot extends SubsystemBase {
  /* If an opponent robot is not on the field, it is placed in a queening position for performance. */
  public static final Pose2d[] ROBOT_QUEENING_POSITIONS = new Pose2d[] {
    new Pose2d(-6, 0, new Rotation2d()),
    new Pose2d(-5, 0, new Rotation2d()),
    new Pose2d(-4, 0, new Rotation2d()),
    new Pose2d(-3, 0, new Rotation2d()),
    new Pose2d(-2, 0, new Rotation2d())
    //new Pose2d(-1, 0, new Rotation2d())
  };

  public static final Pose2d[] ROBOTS_STARTING_POSITIONS = new Pose2d[] {
    new Pose2d(15, 6, Rotation2d.fromDegrees(180)),
    new Pose2d(15, 4, Rotation2d.fromDegrees(180)),
    new Pose2d(15, 2, Rotation2d.fromDegrees(180)),
    new Pose2d(1.6, 6, new Rotation2d()),
    new Pose2d(1.6, 4, new Rotation2d())
    //new Pose2d(1.6, 2, new Rotation2d())
  };

  // you can create as many opponent robots as you needs
  public static final AdditionalSimRobot[] instances = new AdditionalSimRobot[2];
  public static void startOpponentRobotSimulations() {
    try {
      // Robot ID 0 is the main robot, so start counting at 1
      instances[0] = new AdditionalSimRobot(1);
      instances[0].buildBehaviorChooser(
              // PathPlannerPath.fromPathFile("opponent robot cycle path 0"),
              // Commands.none(),
              // PathPlannerPath.fromPathFile("opponent robot cycle path 0 backwards"),
              // Commands.none(),
              new XboxController(1));

      instances[1] = new AdditionalSimRobot(2);
      instances[1].buildBehaviorChooser(
              // PathPlannerPath.fromPathFile("opponent robot cycle path 1"),
              // //instances[1].shootAtSpeaker(),
              // Commands.none(),
              // PathPlannerPath.fromPathFile("opponent robot cycle path 1 backwards"),
              // Commands.none(),
              new XboxController(2));
    } catch (Exception e) {
      DriverStation.reportError("Failed to load opponent robot simulation paths, error: " + e.getMessage(), false);
    }
  }

  @Override
  public void periodic() {
    Pose2d[] poses = new Pose2d[]{
      instances[0].swerveDriveSim.getSimulatedDriveTrainPose(),
      instances[1].swerveDriveSim.getSimulatedDriveTrainPose()
    };
    Logger.recordOutput("FieldSimulation/AdditionalRobots", poses);
  }
  
  /** Build the behavior chooser of this opponent robot and send it to the dashboard */
  public void buildBehaviorChooser(
        // PathPlannerPath segment0,
        // Command toRunAtEndOfSegment0,
        // PathPlannerPath segment1,
        // Command toRunAtEndOfSegment1,
        XboxController joystick) {
    SendableChooser<Command> behaviorChooser = new SendableChooser<>();
    final Supplier<Command> disable =
        () -> Commands.runOnce(() -> swerveDriveSim.setSimulationWorldPose(queeningPose), this)
                      .andThen(Commands.runOnce(
                        () -> swerveDriveSim.setRobotSpeeds(new ChassisSpeeds())))
                          // () -> swerveDriveSim.runChassisSpeeds(new ChassisSpeeds(),
                          //                                       new Translation2d(),
                          //                                       false, false)))
                      .ignoringDisable(true);

    // Option to disable the robot
    behaviorChooser.setDefaultOption("Disable", disable.get());

    // Option to auto-cycle the robot
    // behaviorChooser.addOption(
    //     "Auto Cycle", getAutoCycleCommand(segment0, toRunAtEndOfSegment0,
    //                                            segment1, toRunAtEndOfSegment1));

    // Option to manually control the robot with a joystick
    behaviorChooser.addOption("Joystick Drive", joystickDrive(joystick));

    // Schedule the command when another behavior is selected
    behaviorChooser.onChange((Command::schedule));

    // Schedule the selected command when teleop starts
    RobotModeTriggers.teleop()
                     .onTrue(Commands.runOnce(
                        () -> behaviorChooser.getSelected().schedule()));

    // Disable the robot when the user robot is disabled
    RobotModeTriggers.disabled().onTrue(disable.get());

    SmartDashboard.putData("AIRobotBehaviors/Opponent Robot " + id + " Behavior", behaviorChooser);
}

  /** Get the command to auto-cycle the robot relatively */
  private Command getAutoCycleCommand(
        PathPlannerPath segment0,
        Command toRunAtEndOfSegment0,
        PathPlannerPath segment1,
        Command toRunAtEndOfSegment1) {
    final SequentialCommandGroup cycle = new SequentialCommandGroup();
    final Pose2d startingPose = new Pose2d(
            segment0.getStartingDifferentialPose().getTranslation(),
            segment0.getIdealStartingState().rotation());

    cycle.addCommands(opponentRobotFollowPath(segment0)
            .andThen(toRunAtEndOfSegment0)
            .withTimeout(10));

    cycle.addCommands(opponentRobotFollowPath(segment1)
            .andThen(toRunAtEndOfSegment1)
            .withTimeout(10));

    return cycle.repeatedly()
            .beforeStarting(Commands.runOnce(() -> swerveDriveSim.setSimulationWorldPose(
                    FieldMirroringUtils.toCurrentAlliancePose(startingPose))));
  }

  private final Pose2d queeningPose;
  private final int id;
  private final SwerveDriveSimulation swerveDriveSim;

  public AdditionalSimRobot(int id) {
    this.id = id;
    this.queeningPose = ROBOT_QUEENING_POSITIONS[this.id-1];
    
    // Drive Sim member variables
    DriveTrainSimulationConfig dtc = DriveTrainSimulationConfig.Default()
      .withRobotMass(Kilograms.of(50))
      .withCustomModuleTranslations(getModuleTranslations())
      .withGyro(COTS.ofPigeon2())
      .withSwerveModule(new SwerveModuleSimulationConfig(
                        DCMotor.getKrakenX60(1),
                        DCMotor.getFalcon500(1),
                        TunerConstants.FrontLeft.DriveMotorGearRatio,
                        TunerConstants.FrontLeft.SteerMotorGearRatio,
                        Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage),
                        Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage),
                        Meters.of(TunerConstants.FrontLeft.WheelRadius),
                        KilogramSquareMeters.of(TunerConstants.FrontLeft.SteerInertia),
                        1.2));
    swerveDriveSim = new SwerveDriveSimulation(dtc, queeningPose);
    SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSim);
  }

  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
        new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
        new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
        new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
        new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }

  // PathPlanner configuration
  private static final RobotConfig PP_CONFIG = new RobotConfig(
    Kilograms.of(55),
    KilogramSquareMeters.of(8),
    new ModuleConfig(
      Inches.of(2).in(Meters),
      3.5,
      1.2,
      DCMotor.getKrakenX60(1).withReduction(8.14),
      60, 1), // Swerve module config
    new Translation2d(0.7, 0.7),
    new Translation2d(0.7, -0.7),
    new Translation2d(-0.7, 0.7),
    new Translation2d(-0.7, -0.7)
  );

  // PathPlanner PID settings
  private final PPHolonomicDriveController driveController =
          new PPHolonomicDriveController(new PIDConstants(5.0, 0.02),
                                         new PIDConstants(7.0, 0.05));

  /** Follow path command for opponent robots */
  private Command opponentRobotFollowPath(PathPlannerPath path) {
    return new FollowPathCommand(
      path, // Specify the path
      // Provide actual robot pose in simulation, bypassing odometry error
      () -> swerveDriveSim.getSimulatedDriveTrainPose(),
      // Provide actual robot speed in simulation, bypassing encoder measurement error
      () -> swerveDriveSim.getDriveTrainSimulatedChassisSpeedsRobotRelative(),
      // Chassis speeds output
      // (speeds, feedforwards) -> 
      //   swerveDriveSim.runChassisSpeeds(speeds, new Translation2d(), false, false),
      (speeds, _feedforwards) -> swerveDriveSim.setRobotSpeeds(speeds),
      driveController, // Specify PID controller
      PP_CONFIG,       // Specify robot configuration
      // Flip path based on alliance side
      () -> DriverStation.getAlliance()
          .orElse(DriverStation.Alliance.Blue)
          .equals(DriverStation.Alliance.Red),
      this // AIRobotInSimulation is a subsystem; this command should use it as a requirement
    );
  }

  private Command joystickDrive(XboxController joystick) {
    // Obtain chassis speeds from joystick input
    final Supplier<ChassisSpeeds> joystickSpeeds = () -> new ChassisSpeeds(
      -joystick.getLeftY() * swerveDriveSim.maxLinearVelocity().in(MetersPerSecond),
      -joystick.getLeftX() * swerveDriveSim.maxLinearVelocity().in(MetersPerSecond),
      -joystick.getRightX() * swerveDriveSim.maxAngularVelocity().in(RadiansPerSecond));

    // Obtain driverstation facing for opponent driver station
    final Supplier<Rotation2d> opponentDriverStationFacing = () ->
        FieldMirroringUtils.getCurrentAllianceDriverStationFacing()
                           .plus(Rotation2d.fromDegrees(180));

    return Commands.run(() -> {
      // Calculate field-centric speed from driverstation-centric speed
      final ChassisSpeeds fieldCentricSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
              joystickSpeeds.get(),
              FieldMirroringUtils.getCurrentAllianceDriverStationFacing()
                                 .plus(Rotation2d.fromDegrees(180)));
      // Run the field-centric speed
      // Maybe this should be runVelocity(fieldCentricSpeeds) ?
      swerveDriveSim.setRobotSpeeds(fieldCentricSpeeds);
      // swerveDriveSim.runChassisSpeeds(fieldCentricSpeeds,
      //                                 new Translation2d(),
      //                                 true,
      //                                 true);
      }, this)
      // Before the command starts, reset the robot to a position inside the field
      .beforeStarting(() -> swerveDriveSim.setSimulationWorldPose(
              FieldMirroringUtils.toCurrentAlliancePose(ROBOTS_STARTING_POSITIONS[id - 1])));
  }
}