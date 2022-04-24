// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.controller;
import frc.robot.Constants.lift;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.indexer;


public class Robot extends TimedRobot {
  //private final XboxController m_controller = new XboxController(0);
  private final GenericHID m_driveController = new GenericHID(0);
  private final GenericHID m_controller = new GenericHID(1);
  private final Drivetrain m_swerve = new Drivetrain();
  private final Shooter m_shooter = new Shooter();
  private final Limelight m_Limelight = new Limelight();
  private final Lift m_lift = new Lift();
  private final indexer m_indexer = new indexer(
  Constants.indexer.kconveyerFront, 
  Constants.indexer.kconveyerBack, 
  Constants.indexer.kboomArm,
  Constants.indexer.kfrontSensorId,
  Constants.indexer.kbackSensorId
  );
 

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(0.8);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(1);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
  //private final SlewRateLimiter m_shootLimiter = new SlewRateLimiter(3);
  private SendableChooser<Boolean> _fieldrelative = new SendableChooser<>();
  private SendableChooser<Integer> _auto_choice = new SendableChooser<>();
/**
 * Button Map 
 */
  private Map<String,Integer> _buttonMap = new HashMap<>();

  /**
   * Returns True if Controller False if joystick
   */
  private SendableChooser<Boolean> _driveControllerOrJoystick = new SendableChooser<>();
  double g_timer = 0; //jank timer for gyro reset button
  double g_timerS; //a double that represents the time the button was
  Timer auto_timer = new Timer();

  ShuffleboardTab tab = Shuffleboard.getTab("SmartDashboard");

  @Override
  public void robotInit(){
    
    m_swerve.center();
    _fieldrelative.setDefaultOption("Yes",true);
    //_fieldrelative.addOption("Yes", true);

    _fieldrelative.addOption("No", false);
    
    _driveControllerOrJoystick.addOption("Joystick", false);

    _driveControllerOrJoystick.setDefaultOption("controller", true);


    _auto_choice.setDefaultOption("2 Ball Left", 2);
    _auto_choice.addOption("2 ball right ish", 3);
    _auto_choice.addOption("Auto_1", 2);

    //_auto_choice.addOption("Auto_One", 0);
    _auto_choice.addOption("Do Nothing",1);




    SmartDashboard.putData("FieldRelative?", _fieldrelative);
    SmartDashboard.putData("Controller or joystick driving?", _driveControllerOrJoystick);
    SmartDashboard.putData("Auto Mode", _auto_choice);
  }

  @Override
  public void autonomousInit(){
    auto_timer.start();
    auto_timer.reset();
    m_swerve.resetDriveEncoder();
    m_swerve.resetGyro();
     
  }

  @Override
  public void autonomousPeriodic() {
    
    switch(_auto_choice.getSelected()){

      case 0:
        if(auto_timer.get() < 3){
          m_indexer.DriveBackConveyer(0.25,true);
          m_shooter.Spin(1);

        }else if (auto_timer.get() > 3 && auto_timer.get() < 7){

          m_swerve.drive(1, 0, 0, false);
        }
        else {
          m_swerve.drive(0, 0, 0, true);
          m_shooter.Spin(0);
          m_indexer.DriveBackConveyer(0, true);
        }
        
      
        break;


      case 1:
        m_swerve.drive(0, 0, 0, false);
        break;

      case 2:
        
        if(auto_timer.get() < 3){
          m_shooter.Spin(.87);
          m_indexer.DriveBackConveyer(0.4, true);
          m_indexer.DriveFrontConveyer(0.85, true);
          m_swerve.drive(0, 0, 0, true);
          break;
        }
        //autoState = autoEnum.DRIVEBACK;
        else if (auto_timer.get() < 6){
          //SmartDashboard.putNumber("AutoEncoder", m_swerve.DriveEncoder(0));
          m_shooter.Spin(0);
          m_indexer.DriveBackConveyer(0, true);
          m_swerve.drive(2, 0, 0, false);
          m_indexer.DriveFrontConveyer(1, true);
          break;
        }
        
        else if(auto_timer.get() < 9){
          m_swerve.drive(-2, 0, 0.20, false);
          m_indexer.DriveFrontConveyer(0,true);
          break;
        }
        
        else if(auto_timer.get() > 9 ){
          m_swerve.drive(0, 0, 0, false);
          m_indexer.DriveBackConveyer(0.4, true);
          m_shooter.Spin(.90);
          break;
          
        }else if(auto_timer.get() < 11){
          m_indexer.DriveBackConveyer(0, true);
          m_shooter.Spin(0);
        }
      
      case 3:
       
       if(auto_timer.get() < 3){
          m_shooter.Spin(.88);
          m_indexer.DriveBackConveyer(0.4, true);
          m_indexer.DriveFrontConveyer(0.85, true);
          m_swerve.drive(0, 0, 0, true);
          break;
        }
        //autoState = autoEnum.DRIVEBACK;
        else if (auto_timer.get() < 6){
          //SmartDashboard.putNumber("AutoEncoder", m_swerve.DriveEncoder(0));
          m_shooter.Spin(0);
          m_indexer.DriveBackConveyer(0, true);
          m_swerve.drive(2, 0, 0, false);
          m_indexer.DriveFrontConveyer(1, true);
          break;
        }
      
        else if(auto_timer.get() < 9){
          m_swerve.drive(-1.5, 0, 0.20, false);
          m_indexer.DriveFrontConveyer(0,true);
          break;
        }
      
        else if(auto_timer.get() > 9 ){
          m_swerve.drive(0, 0, 0, false);
          m_indexer.DriveBackConveyer(0.4, true);
          m_shooter.Spin(.90);
          break;
      
        }else if(auto_timer.get() < 13){
          m_indexer.DriveBackConveyer(0, true);
          m_shooter.Spin(0);
        }

      default:
        break;

    }
    //driveWithJoystick(false);
    
  }

  @Override
  public void teleopInit(){

    

    //change button mappings depending on selected controller.
     if(_driveControllerOrJoystick.getSelected()){
      _buttonMap.put("Rot Axis", 4);
      _buttonMap.put("Drive Y", 1);
      _buttonMap.put("Drive X", 0);
      _buttonMap.put("Gyro Reset", 7);
      _buttonMap.put("Center", 1);
      
      auto_timer.stop();
      auto_timer.reset();
      
    }
    else{ 
      _buttonMap.put("Rot Axis",2);
      _buttonMap.put("Drive Y", 1);
      _buttonMap.put("Drive X", 0);
      _buttonMap.put("Gyro Reset", 5);
      _buttonMap.put("Center", 3);
      auto_timer.stop();
      auto_timer.reset();
      
    }

  }
  
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Shooter RPM", m_shooter.getRotations());
    
    SmartDashboard.putNumber("Shooter Percent", m_shooter.getPercent()*100);

    SmartDashboard.putNumber("Left lift encoder", m_lift.get_LeftEncoder());
    SmartDashboard.putNumber("right lift encoder", m_lift.get_RightEncoder());

    SmartDashboard.putNumber("DriveEncoder", m_swerve.DriveEncoder(0));

    //axis controlled subsytems 
    if(!m_controller.getRawButton(controller.button_Y)){
      m_shooter.Spin(m_controller.getRawAxis(controller.right_Trigger) * 0.85);
    }else{
      m_shooter.Spin(-0.2);
    }
    


    if(!m_driveController.getRawButton(controller.right_Bumper)){
      driveWithJoystick(_fieldrelative.getSelected());
    }else{
      driveWithLimeLight(_fieldrelative.getSelected());
    }
    
    if (m_driveController.getRawButton(controller.button_Y)){
      m_Limelight.CamSwitch();
    }
    
   
    if(m_controller.getRawButton(controller.button_A)){
      m_lift.front_left_power(MathUtil.applyDeadband(m_controller.getRawAxis(controller.left_Yaxis),0.1));
      m_lift.front_right_power(MathUtil.applyDeadband(m_controller.getRawAxis(controller.left_Yaxis),0.1));
      m_lift.left_accuate(MathUtil.applyDeadband(m_controller.getRawAxis(controller.left_Xaxis),0.1));
      m_lift.right_accuate(MathUtil.applyDeadband(m_controller.getRawAxis(controller.left_Xaxis),0.1));
    }else{
      m_lift.front_left_power(MathUtil.applyDeadband(m_controller.getRawAxis(controller.left_Yaxis),0.1));
      m_lift.front_right_power(MathUtil.applyDeadband(m_controller.getRawAxis(controller.right_Yaxis),0.1));
      m_lift.left_accuate(MathUtil.applyDeadband(m_controller.getRawAxis(controller.left_Xaxis),0.1));
      m_lift.right_accuate(MathUtil.applyDeadband(m_controller.getRawAxis(controller.right_Xaxis),0.1));
    }

    //m_lift.back_power(m_controller.getRawAxis(controller.left_Yaxis));
  
    if (m_driveController.getRawButton(_buttonMap.get("Gyro Reset"))){
      m_swerve.resetGyro();
    }
    if(m_controller.getRawButton(controller.right_Bumper)){
      m_lift.resetEncoders();
    }
    
    if(m_controller.getRawButton(controller.button_A) && (m_controller.getPOV() == 0)){
      m_lift.back_left_power(1);
      m_lift.back_right_power(1);
    }else if(m_controller.getRawButton(controller.button_A) && m_controller.getPOV() == 180){
      m_lift.back_left_power(-1);
      m_lift.back_right_power(-1);
    }else {
      switch(m_controller.getPOV()){
        case -1:
          m_lift.back_left_power(0);
          m_lift.back_right_power(0);
          break;
        case 0:
          m_lift.back_left_power(1);
          break;
        case 45:
          m_lift.back_left_power(1);
          m_lift.back_right_power(1);
          break;
        case 90:
          m_lift.back_right_power(1);
          break;
        case 180:
          m_lift.back_right_power(-1);
          break;
        
        case 225:
          m_lift.back_right_power(-1);
          m_lift.back_left_power(-1);
          break;
        
        case 270:
          m_lift.back_left_power(-1);
          break;
        
        default:
          m_lift.back_left_power(0);
          m_lift.back_right_power(0);
  
      }
    }
    
    


    // if(m_driveController.getRawButton(1)){
    //   m_swerve.drive(4, 0, 0, true);
    // }    

    // if (m_controller.getRawButton(2)){
    //   m_shooter.Spin(1);
    // }
    // else{
    //   m_shooter.Spin(0);
    // }

    if(m_controller.getRawButton(3)){
      m_indexer.DriveBackConveyer(0.50,true);
    }else{
      m_indexer.DriveBackConveyer(0,true);
    }
    if(m_controller.getRawButton(4)){
      m_indexer.DriveBackConveyer(0.50,false);
    }else{
      //m_indexer.DriveBackConveyer(0);
    }



    if(m_controller.getRawButton(10)){
      m_indexer.DriveBoom(0.50, false);
    }
    else if(m_controller.getRawButton(9)){
      m_indexer.DriveBoom(0.5, true);
    }
    else{
      m_indexer.DriveBoom(0, false);
    }

    if (m_controller.getRawButton(5)){
      m_indexer.DriveFrontConveyer(0.75,false);
    }
    else if(m_controller.getRawButton(6)){
      m_indexer.DriveFrontConveyer(0.75, true);
    }
     else{
      m_indexer.DriveFrontConveyer(0,false);
    }


    //SmartDashboard.putNumber("g_timer", g_timer);
    //artDashboard.putNumber("gyro reset difference", g_timer - g_timerS);
    SmartDashboard.putNumber("Gyro", m_swerve.getGyro().getDegrees());
    //SmartDashboard.putNumber("Boom rotations", m_indexer.boomEncoder());
    SmartDashboard.putNumber("Front Index sens", m_indexer.GetfrontSensor());
    SmartDashboard.putNumber("Back Index sens", m_indexer.getBackSensor());
    
    
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
       - m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_driveController.getRawAxis(_buttonMap.get("Drive Y")), 0.2))
            * Drivetrain.kMaxSpeed;
    
    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value mwhen we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_driveController.getRawAxis(_buttonMap.get("Drive X")), 0.2))
            * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(m_driveController.getRawAxis(_buttonMap.get("Rot Axis")), 0.2))
            * Drivetrain.kMaxAngularSpeed;

    //SmartDashboard.putNumber("rotation", rot);
    //SmartDashboard.putNumber("xSpeed",xSpeed);
    //SmartDashboard.putNumber("ySpeed", ySpeed);
    
    //SmartDashboard.putNumberArray("Encoders",m_swerve.getAllEncoders());
    //SmartDashboard.putNumber("POV", m_controller.getPOV(0));
    //SmartDashboard.putData("Encoders", m_swerve.);
    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
  }
  private void driveWithLimeLight(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
       - m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_driveController.getRawAxis(_buttonMap.get("Drive Y")), 0.2))
            * Drivetrain.kMaxSpeed;
    
    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value mwhen we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_driveController.getRawAxis(_buttonMap.get("Drive X")), 0.2))
            * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
         m_Limelight.BrotationPID() * Drivetrain.kMaxAngularSpeed;

    //SmartDashboard.putNumber("rotation", rot);
    //SmartDashboard.putNumber("xSpeed",xSpeed);
    //SmartDashboard.putNumber("ySpeed", ySpeed);
    //SmartDashboard.putNumber("Shooter Percent", m_controller.getRawAxis(3) * 100);
    //SmartDashboard.putNumberArray("Encoders",m_swerve.getAllEncoders());
    //SmartDashboard.putNumber("POV", m_controller.getPOV(0));
    //SmartDashboard.putData("Encoders", m_swerve.);
    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
  }
}
