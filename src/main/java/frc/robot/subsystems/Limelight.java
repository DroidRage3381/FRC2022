// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");  

  private PIDController  seekPID = new PIDController(0.01, 0, 0);

  public Limelight() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
  }

    double Kseekp = -0.05;
    double min_command = 0.1;
    double steering_adjust = 0.0;

 public double Xvalue(){
    double x =tx.getDouble(0);
    return x;
  }

  public double RotationPID(){

    double x = Xvalue();
    double heading_error = -x;
    
    
    if (x > 0)
    {
      steering_adjust = Kseekp * heading_error - min_command;
    }
    else if (x < 0)
    {
      steering_adjust = Kseekp * heading_error + min_command;
    }
    return steering_adjust;
  }

  /**
   * 
   * @return calculated PID value for distance from center of target
   */
  public double BrotationPID(){

     return seekPID.calculate(Xvalue(), 0);

  }
  

  public void CamSwitch(){
      boolean cam_mode = true;
    if (cam_mode){
      //camera mode
      cam_mode = !cam_mode;
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
    }
    else{
      //vision proccess mode
      cam_mode = !cam_mode;
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    }
  }  
}
