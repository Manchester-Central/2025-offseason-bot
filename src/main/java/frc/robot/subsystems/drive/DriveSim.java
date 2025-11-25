package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
// import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.generated.TunerConstants;

public class DriveSim extends DriveIO {
  // SimGyro
  // private final GyroIO gyroIO;
  // private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  // Modules
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR

  // Sim Setup Related
  private final Supplier<Pose2d> poseSetupSupplier;
  private final SwerveDriveSimulation swerveDriveSim;

  public DriveSim(Supplier<Pose2d> initPose) {
    // Explicitly don't call super here, it will re-init PathPlanner and crash
    // this.gyroIO = new GyroIOSim(new GyroSimulation(0.0, 0.05));
    this.poseSetupSupplier = initPose;

    DriveTrainSimulationConfig dtc = DriveTrainSimulationConfig.Default()
        .withRobotMass(Kilograms.of(50))
        .withCustomModuleTranslations(DriveIO.getModuleTranslations())
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
    swerveDriveSim = new SwerveDriveSimulation(dtc, this.poseSetupSupplier.get());
    SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSim);

    //
    this.modules[0] = new Module(new ModuleIOTalonFXSim(TunerConstants.FrontLeft, swerveDriveSim.getModules()[0]), 0, TunerConstants.FrontLeft);
    this.modules[1] = new Module(new ModuleIOTalonFXSim(TunerConstants.FrontRight, swerveDriveSim.getModules()[1]), 1, TunerConstants.FrontRight);
    this.modules[2] = new Module(new ModuleIOTalonFXSim(TunerConstants.BackLeft, swerveDriveSim.getModules()[2]), 2, TunerConstants.BackLeft);
    this.modules[3] = new Module(new ModuleIOTalonFXSim(TunerConstants.BackRight, swerveDriveSim.getModules()[3]), 3, TunerConstants.BackRight);
  }

  @Override
  public void resetPose() {
    swerveDriveSim.setSimulationWorldPose(this.poseSetupSupplier.get());
  }

  @Override
  public Pose2d getPose() {
    return swerveDriveSim.getSimulatedDriveTrainPose();
  }

  @Override
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  @Override
  public void setPose(Pose2d pose) {
    swerveDriveSim.setSimulationWorldPose(pose);
  }

  @Override
  public void runVelocity(ChassisSpeeds speeds) {
    // *pukes*
    swerveDriveSim.setRobotSpeeds(ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getRotation()));
  }

  @Override
  public ChassisSpeeds getChassisSpeeds() {
    return swerveDriveSim.getDriveTrainSimulatedChassisSpeedsRobotRelative();
  }

  @Override
  public void stop() {
    swerveDriveSim.setRobotSpeeds(new ChassisSpeeds());
  }

  @Override
  public void stopWithX() {
    // Maybe add a lot of mass?
    swerveDriveSim.setRobotSpeeds(new ChassisSpeeds());
  }

  @Override
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    // Pass
  }

  @Override
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  @Override
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }
}
