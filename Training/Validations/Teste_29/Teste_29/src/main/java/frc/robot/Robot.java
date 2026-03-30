// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.XboxController;

import java.lang.ModuleLayer.Controller;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;


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
  private CANSparkMax ITK1;                         // TRAÇÃO MOTOR SUPERIOR DIREITO
  private CANSparkMax ITK2;                         // TRAÇÃO MOTOR INFERIOR ESQUERDO
  private CANSparkMax ITK3;                         // TRAÇÃO MOTOR INFERIOR DIREITO

  private static XboxController controller;

  boolean CTR=false, TG=false;
  @Override
  public void robotInit() 
  {
    controller = new XboxController(0);

    ITK1 = new CANSparkMax(1, MotorType.kBrushless);
    ITK2 = new CANSparkMax(4, MotorType.kBrushless);
    ITK3 = new CANSparkMax(3, MotorType.kBrushless);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() 
  {
    double Velocit=0, Ve=0;

    Velocit = controller.getRawAxis(3);
    Ve = controller.getRawAxis(2);

    if(controller.getAButton()==true)
    {
      Velocit = controller.getRawAxis(3)*(-1);
      Ve = controller.getRawAxis(2)*(-1);
    }
    if(controller.getRawButton(5)==true && CTR==false)
    {
      TG=!TG;
      CTR=true;
    }
    if(controller.getRawButton(5)==false && CTR==true)
    {
      CTR=false;
    }
    if(TG=true)
    {
      ITK1.setIdleMode(IdleMode.kBrake);
      ITK2.setIdleMode(IdleMode.kBrake);
      ITK3.setIdleMode(IdleMode.kBrake);
    }
    else
    {
      ITK1.setIdleMode(IdleMode.kCoast);
      ITK2.setIdleMode(IdleMode.kCoast);
      ITK3.setIdleMode(IdleMode.kCoast);
    }

    ITK1.set(Velocit);
    ITK2.set(Velocit);
    ITK3.set(Ve);
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
