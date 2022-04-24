// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.kauailabs.navx.frc.AHRS;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  public static final double kMaxSpeed =12.0; //
  public static final double kMaxAngularSpeed = 6 * Math.PI; // 1/2 rotation per second

  // private final Translation2d m_frontLeftLocation = new Translation2d(0.283972,
  // 0.283972);
  // private final Translation2d m_frontRightLocation = new
  // Translation2d(0.283972, -0.283972);
  // private final Translation2d m_backLeftLocation = new Translation2d(-0.283972,
  // 0.283972);
  // private final Translation2d m_backRightLocation = new
  // Translation2d(-0.283972, -0.283972);

  private final Translation2d m_frontLeftLocation = new Translation2d(0.2874, 0.2874);
  private final Translation2d m_frontRightLocation = new Translation2d(0.2874, -0.2874);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.2874, 0.2874);
  private final Translation2d m_backRightLocation = new Translation2d(-0.2874, -0.2874);

  // private ShuffleboardTab Tab = Shuffleboard.getTab("SmartDashboard");
  // private NetworkTableEntry Turn_P = Tab.addPersistent("Turn_P",
  // 0.075).getEntry();
  // private NetworkTableEntry Turn_I = Tab.addPersistent("Turn_I", 0).getEntry();
  // private NetworkTableEntry Turn_D = Tab.addPersistent("Turn_D",
  // 0.05).getEntry();

  // private double[] TurnPIDArray =
  // {Turn_P.getDouble(0.075),Turn_I.getDouble(0),Turn_D.getDouble(0.05)};

  // it goes drivemotor, turnmotor, cancoder
  private final SwerveModule m_frontLeft = new SwerveModule(
      Constants.DriveFalcons.kTalonFXID_FL_Drive,
      Constants.DriveFalcons.kTalonFXID_FL_Turn,
      Constants.Cancoders.kCanCoderId_FL);
  private final SwerveModule m_frontRight = new SwerveModule(
      Constants.DriveFalcons.kTalonFXID_FR_Drive,
      Constants.DriveFalcons.kTalonFXID_FR_Turn,
      Constants.Cancoders.kCanCoderId_FR);
  private final SwerveModule m_backLeft = new SwerveModule(
      Constants.DriveFalcons.kTalonFXID_BL_Drive,
      Constants.DriveFalcons.kTalonFXID_BL_Turn,
      Constants.Cancoders.kCanCoderId_BL);
  private final SwerveModule m_backRight = new SwerveModule(
      Constants.DriveFalcons.kTalonFXID_BR_Drive,
      Constants.DriveFalcons.kTalonFXID_BR_Turn,
      Constants.Cancoders.kCanCoderId_BR);

  // private final AnalogGyro m_gyro = new AnalogGyro(0);

  // private final AHRS m_gyro = new AHRS(Port.kUSB1);
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d());

  public Drivetrain() {
    m_gyro.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
     SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  // allign all the motors parellel to eachother, perpendicular to the front
  public void center() {
    var centerstate = new SwerveModuleState();
    m_frontLeft.setDesiredState(centerstate);
    m_frontRight.setDesiredState(centerstate);
    m_backLeft.setDesiredState(centerstate);
    m_backRight.setDesiredState(centerstate);

  }
  /**
   * 
   * @param integer from 0 - 3, 0 being front left, 1: front right, 3:backleft
   * @return Encoder in encoder units, the resolution is 2048, defaults to front left.
   */
  public double DriveEncoder(int motor){
    switch(motor){
      case 0:
        return m_frontLeft.DriveEncoder();
        
      case 1:
        return m_frontRight.DriveEncoder();
      
      case 2:
        return m_backLeft.DriveEncoder();

      case 3:
        return m_backRight.DriveEncoder();

        default:
          return m_frontLeft.DriveEncoder();
    
    }
  }
  public void resetDriveEncoder(){
    m_frontLeft.resetDriveEncoder();
  }

  public Rotation2d getGyro() {
    return m_gyro.getRotation2d();
  }

  public void resetGyro() {
    m_gyro.reset();
  }

  /***
   * 
   * @param module starts at zero being front left, 1 is FR, 2 is BL, 3 is BR.
   * @return returns the specified modules encoder value, defaults to 0.0
   */
  // public double getModuleEnocder(int module) {
  //   switch (module) {
  //     case 0:
  //       return m_frontLeft.coder_value();

  //     case 1:
  //       return m_frontRight.coder_value();

  //     case 2:
  //       return m_backLeft.coder_value();

  //     case 3:
  //       return m_backRight.coder_value();

  //     default:
  //       return 0.0;
  //   }

  //}

  /**
   * 
   * @return
   */
  // public double[] getAllEncoders() {
  //   double[] encoders = { m_frontLeft.coder_value(), m_frontRight.coder_value(), m_backLeft.coder_value(),
  //       m_backRight.coder_value() };
  //   return encoders;
  // }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState());
  }
}
