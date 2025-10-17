package frc.robot.subsystems.multisim;

import edu.wpi.first.math.Pair;
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
  /* What the heck is a queening position? */
  public static final Pose2d[] ROBOT_QUEENING_POSITIONS = new Pose2d[] {
    new Pose2d(-2.0, 0, new Rotation2d()),
    new Pose2d(-3.5, 0, new Rotation2d()),
    new Pose2d(-5.0, 0, new Rotation2d()),
    new Pose2d(-6.5, 0, new Rotation2d()),
    new Pose2d(-8.0, 0, new Rotation2d())
  };

  public static final Pose2d[] ROBOTS_STARTING_POSITIONS = new Pose2d[] {
    // 2 allies
    new Pose2d(7.2, 6, Rotation2d.fromDegrees(180)),
    new Pose2d(7.2, 4, Rotation2d.fromDegrees(180)),
    // then 3 enemies
    new Pose2d(9.8, 2, new Rotation2d()),
    new Pose2d(9.8, 6, new Rotation2d()),
    new Pose2d(9.8, 4, new Rotation2d())
  };

  // you can create as many opponent robots as you needs
  public static final AdditionalSimRobot[] instances = new AdditionalSimRobot[5];
  public static void startOpponentRobotSimulations() {
    int idx = 0;
    try {
      // Robot ID 0 is the main robot, so start counting at 1
      for (idx = 0; idx < instances.length; idx++) {
        int id = idx+1;
        instances[idx] = new AdditionalSimRobot(id);
        instances[idx].buildBehaviorChooser(
                PathPlannerPath.fromPathFile("opponent robot cycle path "+idx),
                Commands.none(),
                PathPlannerPath.fromPathFile("opponent robot cycle path "+idx+" backwards"),
                Commands.none(),
                new XboxController(id));
      }
    } catch (Exception e) {
      DriverStation.reportError("Failed to load opponent robot "+idx+" simulation paths, error: " + e.getMessage(), false);
    }
  }

  /**
   * Object specific member variables and functions
   */

  private final Pose2d queeningPose;
  private final int id;
  private final SwerveDriveSimulation swerveDriveSim;

  // PathPlanner PID settings
  private final PPHolonomicDriveController driveController =
          new PPHolonomicDriveController(new PIDConstants(5.0, 0.02),
                                         new PIDConstants(7.0, 0.05));
  // PathPlanner configuration
  private static final RobotConfig PP_CONFIG = new RobotConfig(
    Kilograms.of(55),
    KilogramSquareMeters.of(8),
    new ModuleConfig(
      Inches.of(2).in(Meters),
      3.5, // TODO: how to pull?
      1.2, // TODO: how to pull?
      DCMotor.getKrakenX60(1).withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio), // presumably all the same
      TunerConstants.FrontLeft.SlipCurrent, // TODO: double check right field
      1), // Swerve module config
    getModuleTranslations()
  );

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

  @Override
  public void periodic() {
    Pose2d[] poses = new Pose2d[]{
      instances[0].swerveDriveSim.getSimulatedDriveTrainPose(),
      instances[1].swerveDriveSim.getSimulatedDriveTrainPose()
    };
    Logger.recordOutput("FieldSimulation/AdditionalRobots", poses);
  }

  // Collects the swerve pod locations from the auto generated TunerConstants
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
        new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
        new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
        new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
        new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }

  /**
   * Build the behavior chooser of this opponent robot and send it to the dashboard,
   * cycle commands will repeat from the start when they expire.
   * @param segment0
   * @param toRunAtEndOfSegment0
   * @param segment1
   * @param toRunAtEndOfSegment1
   * @param joystick
   */
  public void buildBehaviorChooser(
        PathPlannerPath segment0,
        Command toRunAtEndOfSegment0,
        PathPlannerPath segment1,
        Command toRunAtEndOfSegment1,
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
    behaviorChooser.addOption(
        "Auto Cycle", getAutoCycleCommand(segment0, toRunAtEndOfSegment0,
                                               segment1, toRunAtEndOfSegment1));

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

    cycle.addCommands(simRobotFollowPath(segment0)
            .andThen(toRunAtEndOfSegment0)
            .withTimeout(10));

    cycle.addCommands(simRobotFollowPath(segment1)
            .andThen(toRunAtEndOfSegment1)
            .withTimeout(10));

    return cycle.repeatedly()
            .beforeStarting(Commands.runOnce(() -> swerveDriveSim.setSimulationWorldPose(
                    FieldMirroringUtils.toCurrentAlliancePose(startingPose))));
  }

  /**
   * Sets up a sequential command group to follow a path, and then execute a command.
   * Takes an arbitrary number of pairs, and repeats the cycle when done.
   *
   * @param cmd list of pairs of actions
   * @return the compiled sequential command
   */
  public Command createPathSequence(Pair<PathPlannerPath, Command>... cmd) {
    if (cmd.length == 0) throw new IllegalArgumentException();
    final SequentialCommandGroup cycle = new SequentialCommandGroup();

    for (var p : cmd) {
      cycle.addCommands(simRobotFollowPath(p.getFirst())
            .andThen(p.getSecond())
            .withTimeout(10));
    }

    return cycle.repeatedly();

    // return cycle.repeatedly()
    //         .beforeStarting(Commands.runOnce(() -> swerveDriveSim.setSimulationWorldPose(
    //         FieldMirroringUtils.toCurrentAlliancePose(startingPose))));
  }

  /**
   * Follow path command for additional robots during simulated autonomous
   * @param path 
   * @return the command segment to be run
   */
  private Command simRobotFollowPath(PathPlannerPath path) {
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

  /**
   * Generates a player driver command for an additional robot
   *
   * @param joystick object associated with a specific controller ID
   * @return cmd to be run during teleop
   */
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
    // Overly fancy trick from IronMaple that amounts to "face away" and then
    // "add 180 degrees so you're actually facing me"

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
