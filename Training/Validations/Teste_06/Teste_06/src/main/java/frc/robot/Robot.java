// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.Encoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Servo;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  private CANSparkMax MSD;
  private CANEncoder ESD;
  private PWM PWM_Pin5;
  private PWM motorPWM;
  Encoder Encoder_Ext = new Encoder(7,8);  // Leitura de Encoder
  private Servo Servo_Pin7;

  @Override
  public void robotInit()
  {
    Encoder_Ext.reset();

    MSD = new CANSparkMax(6, MotorType.kBrushless);
    ESD = MSD.getEncoder();
    
    PWM_Pin5 = new PWM(5);
    Servo_Pin7 = new Servo(7);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit(){}

  @Override
  public void teleopPeriodic() 
  {
    int EncoderValue = Encoder_Ext.get();
    double CANencoderPosi = ESD.getPosition();
    System.out.println("Encoder Externo Position: " + EncoderValue);
    System.out.println("Encoder Interno Position: " + CANencoderPosi);
    /*
    PWM_Pin5.setSpeed(0.8);   // Ajusta o Duty Cicle para 80%
    Timer.delay(2);
    PWM_Pin5.setSpeed(0.3);   // Ajusta o Duty Cicle para 30%
    Timer.delay(2);
    */
    Servo_Pin7.set(1);  // Ajusta a posição do Servo para 180°/0°
    Timer.delay(2);
    Servo_Pin7.set(0);  // Ajusta a posição do Servo para 0°/180°
    Timer.delay(2);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
