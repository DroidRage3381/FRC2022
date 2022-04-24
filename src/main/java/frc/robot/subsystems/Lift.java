// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.lift;

public class Lift extends SubsystemBase {
  /** Creates a new Lift. */

   private final CANSparkMax frontRight = new CANSparkMax(lift.kFrontRight_id, MotorType.kBrushless);
   private final CANSparkMax frontLeft = new CANSparkMax(lift.kFrontLeft_id, MotorType.kBrushless);
   private final CANSparkMax backRight = new CANSparkMax(lift.kBackRight_id, MotorType.kBrushless);
   private final CANSparkMax backLeft = new CANSparkMax(lift.kBackLeft_id, MotorType.kBrushless);

   private final TalonSRX liftTalon_1 = new TalonSRX(lift.kLiftTalon_1);
   private final TalonSRX liftTalon_2 = new TalonSRX(lift.kLiftTalon_2);

   private final Encoder leftEncoder = new Encoder(0,1);
   private final Encoder RightEncoder = new Encoder(2,3);
   private final int limit = 645;

   
  
  public Lift() {
    
    
  }
  public double get_LeftEncoder(){
    return leftEncoder.get();
  }
  public double get_RightEncoder(){
    return RightEncoder.get();
  }
  public void resetEncoders(){
    leftEncoder.reset();
    RightEncoder.reset();
  }
  public void front_right_power(double p){
    frontRight.set(p);
  }

  public void front_left_power(double p){
    frontLeft.set(p);
  }

  public void back_right_power(double p){
    backRight.set(p);
  }

  public void back_left_power(double p){
    backLeft.set(p);
  }
  
  public void back_power(double p){
    backLeft.set(p);
    backRight.set(p);
  
  }
  public void allpower(double p){
    backLeft.set(p);
    backRight.set(p);
    front_accuate(p);
  }
  public void front_accuate(double p){
    front_accuate(p);
    left_accuate(p);
  }

  public void left_accuate(double p){
    if(Math.abs(get_LeftEncoder()) > 475 && p < 0){
        p = 0;
    }else {

      liftTalon_1.set(TalonSRXControlMode.PercentOutput, p);

    }
    //liftTalon_1.set(TalonSRXControlMode.PercentOutput, p);
  }
  public void right_accuate(double p){
    if(Math.abs(get_RightEncoder()) > 475 && p < 0){
      p = 0;
      //liftTalon_2.set(TalonSRXControlMode.PercentOutput, p);

    }else{
      liftTalon_2.set(TalonSRXControlMode.PercentOutput, p);
    }
    //liftTalon_2.set(TalonSRXControlMode.PercentOutput, p);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
