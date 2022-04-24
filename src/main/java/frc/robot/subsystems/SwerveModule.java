// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SwerveModule {
  private static final double kWheelRadius = 0.0508;
  private static final int kEncoderResolution = 4096;
  

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared
  

  private final TalonFX m_driveMotor;
  private final TalonFX m_turningMotor; 
  private final CANCoder m_turningEncoder;
  

  private CANCoderConfiguration config = new CANCoderConfiguration();
  private TalonFXConfiguration  mt_config = new TalonFXConfiguration();
  
  
  

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(0.07, 0, 0);
  
  // Gains are for example purposes only - must be determined for your own robot!
  
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          0.6,
          0,
          0,
           new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));// pld I and D are .11 and .018
  
  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.1, 0);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(0, 0);// old values : .143 and .026

  


  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorID PWM output for the drive motor.
   * @param turningMotorID PWM output for the turning motor.
   * @param turningEncoderID CAN ID for Turning CANCoder
   */
  public SwerveModule(
      int driveMotorId,
      int turningMotorID,
      int turningEncoderID) {
    m_driveMotor =  new TalonFX(driveMotorId,"Drivetrain");
    m_turningMotor =  new TalonFX(turningMotorID,"Drivetrain");
    
    m_turningEncoder = new CANCoder(turningEncoderID,"Drivetrain");


      
    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    
    //config.sensorCoefficient = 2 * Math.PI/4096.0;
    //config.sensorTimeBase = SensorTimeBase.PerSecond;
    //config.unitString = "rad";
    //config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
    
    //m_driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
    
    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
   // m_turningEncoder.configAllSettings(config);
   // m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);
    
        m_turningMotor.setNeutralMode(NeutralMode.Brake);
        m_driveMotor.setNeutralMode(NeutralMode.Brake);

        mt_config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        //m_turningMotor.configAllSettings(mt_config);
    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }


  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    double driveMPS = (m_driveMotor.getSelectedSensorVelocity()/2048) * (2 * Math.PI * kWheelRadius);
    return new SwerveModuleState(driveMPS, new Rotation2d(Math.toRadians(m_turningEncoder.getAbsolutePosition()) ));
    
  }
  public double DriveEncoder(){
    return m_driveMotor.getSelectedSensorPosition();
  }

  public void resetDriveEncoder(){
    m_driveMotor.setSelectedSensorPosition(0);
  }

  /**
   * 
   * @return returns the swerve modules encoder value
   */
  // public double coder_value(){
  //   return m_turningEncoder.getAbsolutePosition();
  // }


  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   * 
   */
  public void setDesiredState(SwerveModuleState desiredState ) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(Math.toRadians(m_turningEncoder.getAbsolutePosition())));
    //double currentheading = m_turningEncoder.getAbsolutePosition();

    double driveMPS = (m_driveMotor.getSelectedSensorVelocity()/2048) * (2 * Math.PI * kWheelRadius);
    // Calculate the drive output from the drive PID controller.

    
    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turningPIDController.calculate(Math.toRadians(m_turningEncoder.getAbsolutePosition()), state.angle.getRadians());

    final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);
    
    // if (Math.abs(m_turningEncoder.getAbsolutePosition() - state.angle.getDegrees()) < 25){
    //   final double driveOutput = m_drivePIDController.calculate(driveMPS, state.speedMetersPerSecond);
    //   final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);
      
    //   m_driveMotor.set(ControlMode.PercentOutput,(driveOutput + driveFeedforward));
    // }else{
    //   m_driveMotor.set(ControlMode.PercentOutput, 0);
    // }
      

    final double driveOutput = m_drivePIDController.calculate(driveMPS, state.speedMetersPerSecond);
    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);
    m_driveMotor.set(ControlMode.PercentOutput,(driveOutput + driveFeedforward));

    m_turningMotor.set(ControlMode.PercentOutput,(turnOutput + turnFeedforward));
    
  }
}
