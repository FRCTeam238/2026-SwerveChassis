package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

@Logged
public class SwerveModule {
  TalonFX driveMotor;
  @NotLogged
  SparkMax turnMotor;
  @NotLogged
  SparkClosedLoopController turningPIDController;
  @NotLogged
  AbsoluteEncoder turnEncoder;

  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  public SwerveModule(int driveID, int turnID) {

    driveMotor = new TalonFX(driveID);
    turnMotor = new SparkMax(turnID, MotorType.kBrushless);

    SparkMaxConfig turnConfig = new SparkMaxConfig();
    turnConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(
            SwerveModuleConstants.turnP,
            SwerveModuleConstants.turnI,
            SwerveModuleConstants.turnD
            )
        .positionWrappingMinInput(SwerveModuleConstants.kTurningEncoderPositionPIDMinInput)
        .positionWrappingMaxInput(SwerveModuleConstants.kTurningEncoderPositionPIDMaxInput)
        .positionWrappingEnabled(true).
        feedForward.kV(SwerveModuleConstants.turnFF);

    turnEncoder = turnMotor.getAbsoluteEncoder();
    turnConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(SwerveModuleConstants.turningCurrentLimit);

    turnConfig.absoluteEncoder
        .positionConversionFactor(SwerveModuleConstants.kTurningEncoderPositionFactor)
        .velocityConversionFactor(SwerveModuleConstants.kTurningEncoderVelocityFactor)
        .inverted(true);
    turningPIDController = turnMotor.getClosedLoopController();

    turnConfig.signals.primaryEncoderPositionPeriodMs(100);
    turnConfig.signals.absoluteEncoderPositionPeriodMs(10);
    turnConfig.signals.absoluteEncoderVelocityPeriodMs(200);

    var config = new TalonFXConfiguration();
    config.Slot0.kP = SwerveModuleConstants.driveP;
    config.Slot0.kI = SwerveModuleConstants.driveI;
    config.Slot0.kD = SwerveModuleConstants.driveD;
    config.Slot0.kV = SwerveModuleConstants.driveFF;
    config.Slot0.kS = SwerveModuleConstants.driveKs;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.StatorCurrentLimit = SwerveModuleConstants.driveCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.Feedback.SensorToMechanismRatio = 1 / SwerveModuleConstants.kDriveMetersPerRev;
    driveMotor.getConfigurator().apply(config);
    driveMotor.getVelocity().setUpdateFrequency(100); // Set update frequency to 100 Hert, 10ms
    driveMotor.getPosition().setUpdateFrequency(100); // Set update frequency to 100 Hert, 10ms
    driveMotor.getClosedLoopError().setUpdateFrequency(50);
    driveMotor.getClosedLoopOutput().setUpdateFrequency(50);
    driveMotor.getSupplyVoltage().setUpdateFrequency(20);
    driveMotor.getSupplyCurrent().setUpdateFrequency(20);
    driveMotor.getStatorCurrent().setUpdateFrequency(20);
    driveMotor.optimizeBusUtilization();

    m_desiredState.angle = new Rotation2d(turnEncoder.getPosition());
    turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        driveMotor.getVelocity().getValueAsDouble(), new Rotation2d(turnEncoder.getPosition()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveMotor.getPosition().getValueAsDouble(), new Rotation2d(turnEncoder.getPosition()));
  }

  public SwerveModuleState getDesiredState() {
    return m_desiredState;
  }

  public void setDesiredState(SwerveModuleState state) {
    Rotation2d encoderRotation = new Rotation2d(turnEncoder.getPosition());
    state.optimize(encoderRotation);
    turningPIDController.setSetpoint(
        state.angle.getRadians(), SparkMax.ControlType.kPosition);

    // Scale speed by cosine of angle error. This scales down movement perpendicular
    // to the desired
    // direction of travel that can occur when modules change directions. This
    // results in smoother driving.
    state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();
    var requestedVoltage = new VelocityVoltage(state.speedMetersPerSecond);
    driveMotor.setControl(requestedVoltage);

    m_desiredState = state;
  }

  public void resetEncoders() {
    driveMotor.setPosition(0);
  }

  public static class SwerveModuleConstants {
    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static int turningCurrentLimit = 30;
    public static double kTurningEncoderPositionPIDMinInput = 0;
    public static double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor;
    public static double turnP = 1;
    public static double turnI = 0;
    public static double turnD = 0.05;
    public static double turnFF = 0;

    public static int driveCurrentLimit = 80;
    public static double driveP = 5;
    public static double driveI = 0;
    public static double driveD = 0.001;
    public static double driveFF = 2.5;
    public static double driveKs = .07;

    public static double wheelDiameter = Units.inchesToMeters(4.13);//4.13?
    public static double wheelCircumference = wheelDiameter * Math.PI;
    public static double driveRatio = 50. / 14. * 17. / 27. * 45. / 15.; // MK4I L2
    public static double kDriveMetersPerRev = wheelCircumference / driveRatio;
  }
}
