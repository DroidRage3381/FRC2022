// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.lang.Math;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //cancoder Id's
   
    public class Cancoders{
        public static final int kCanCoderId_FL = 2;
        public static final int kCanCoderId_FR = 5;
        public static final int kCanCoderId_BR = 8;
        public static final int kCanCoderId_BL = 11;
    }
    //Drive Falcons
    public class DriveFalcons{
        public static final int kTalonFXID_FL_Drive = 1;
        public static final int kTalonFXID_FL_Turn = 3;
        public static final int kTalonFXID_FR_Drive = 4;
        public static final int kTalonFXID_FR_Turn = 6;
        public static final int kTalonFXID_BR_Drive = 7;
        public static final int kTalonFXID_BR_Turn = 9;
        public static final int kTalonFXID_BL_Drive = 10;
        public static final int kTalonFXID_BL_Turn = 12;

    }
    public class controller { 
        public static final int button_A = 1;
        public static final int button_B = 2;
        public static final int button_X = 3;
        public static final int button_Y = 4;
        public static final int left_Bumper = 5;
        public static final int right_Bumper = 6;
        public static final int button_Back  = 7;
        public static final int button_Start = 8;
        public static final int left_Xaxis = 0;
        public static final int left_Yaxis = 1;
        public static final int left_Trigger = 2;
        public static final int right_Trigger = 3;
        public static final int right_Xaxis = 4;
        public static final int right_Yaxis = 5;
    }

    public class lift{
        public static final int kFrontLeft_id = 21;
        public static final int kFrontRight_id = 22; 
        public static final int kBackRight_id = 23;
        public static final int kBackLeft_id = 24;
        public static final int kLiftTalon_1 = 25;
        public static final int kLiftTalon_2 = 26;
    }
    public class indexer{
        public static final int kboomArm = 41;
        public static final int kconveyerFront = 42;
        public static final int kconveyerBack = 43; 
        public static final int kfrontSensorId = 0;
        public static final int kbackSensorId = 1;
        public static final int kupperBoomLimit = 0;
        public static final int kloverBoomLimit = 50;

    }

    public class shooter{
       public static final int kleftTalon = 31;
       public static final int kRightTalon =32;
    }
   public class autoInfo{
        public static final int wheelRadius = 4;
        public static final double gearRatio = 8.14;
        public static final double distancePerWheelRev = 25.13/39.37;
        public static final double BallDistance = 118/39.37;
        public static final double coderPerMeter = 26143.6049;
        

   }
}
