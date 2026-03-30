// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.XboxController;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
  public static final double ROBOT_MASS = 56; // 32lbs * kg per pound    60 * 0.453592
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(4)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class AutonConstants
  {

    // public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.0005, 0, 0);
    // public static final PIDConstants ANGLE_PID   = new PIDConstants(0.0005, 0, 0);

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.05, 0, 0);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(0.04, 0, 0);

    // public static final PIDConstants TRANSLATION_PID = new PIDConstants(2, 0, 0);
    // public static final PIDConstants ANGLE_PID   = new PIDConstants(2, 0, 0.1);
  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 2; ///2
  }
  
  public static class ConstShutter {
    public static final CANSparkMax shutter_itk = new CANSparkMax(22, MotorType.kBrushless);  //23
    public static final CANSparkMax shutter_disp_sup = new CANSparkMax(21, MotorType.kBrushless);
    public static final CANSparkMax shutter_disp_inf = new CANSparkMax(23, MotorType.kBrushless); //22
    public static final CANSparkMax shutter_inc_right = new CANSparkMax(20, MotorType.kBrushless);
    public static final CANSparkMax shutter_inc_left = new CANSparkMax(24, MotorType.kBrushless);

    public static final SparkPIDController pidShutterLeft = shutter_inc_left.getPIDController();
    public static final SparkPIDController pidShutterRight = shutter_inc_right.getPIDController();
  }

  public static class ConstChassy {
    public static final CANSparkMax itk_Front = new CANSparkMax(14, MotorType.kBrushless);
    public static final CANSparkMax itk_Back1 = new CANSparkMax(15, MotorType.kBrushless);
    public static final CANSparkMax itk_Back2 = new CANSparkMax(16, MotorType.kBrushless);
    public static final CANSparkMax itk_Mid = new CANSparkMax(17, MotorType.kBrushless);
    public static final CANSparkMax turret_Hor = new CANSparkMax(18, MotorType.kBrushless);
    
    public static final DigitalInput sensor_Itk = new DigitalInput(0);

    public static final DigitalOutput btn_modeUp = new DigitalOutput(4);
    public static final DigitalOutput btn_modeDw = new DigitalOutput(8);
    public static final DigitalOutput btn_veloUp = new DigitalOutput(5);
    public static final DigitalOutput btn_veloDw = new DigitalOutput(6);
    public static final DigitalOutput btn_onOff = new DigitalOutput(7);
    public static final DigitalOutput btn_pause = new DigitalOutput(9);


    public static final SparkPIDController pidTurret = turret_Hor.getPIDController();

    public static final Pigeon2 pigeon = new Pigeon2(13);
  }
  public static class ConstController {
      public static final XboxController controllerDriver = new XboxController(0);
      public static final XboxController controllerCoDriver = new XboxController(1);
    
  }
}
