// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenixpro.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import javax.management.timer.TimerMBean;

import com.ctre.phoenix.sensors.CANCoder;




public class Robot extends TimedRobot 
{

  private CANSparkMax TMSE;
  private CANSparkMax TMSD;
  private CANSparkMax TMIE;
  private CANSparkMax TMID;

  private CANSparkMax AMSE;
  private CANSparkMax AMSD;
  private CANSparkMax AMIE;
  private CANSparkMax AMID;

  private CANcoder CCMSE;
  private CANcoder CCMSD;
  private CANcoder CCMIE;
  private CANcoder CCMID;

  private XboxController controller;
  private double pos_des = 0;
  private boolean trava_01 = false;
  private boolean TG_PID = false;


  @Override
  public void robotInit() 
  {
    TMSE = new CANSparkMax(5,MotorType.kBrushless);
    AMSE = new CANSparkMax(6,MotorType.kBrushless);
    CCMSE = new CANcoder(9);

    TMSD = new CANSparkMax(7,MotorType.kBrushless);
    AMSD = new CANSparkMax(8,MotorType.kBrushless);
    CCMSD = new CANcoder(10);

    TMIE = new CANSparkMax(3,MotorType.kBrushless);
    AMIE = new CANSparkMax(4,MotorType.kBrushless);
    CCMIE = new CANcoder(11);

    TMID = new CANSparkMax(1,MotorType.kBrushless);
    AMID = new CANSparkMax(2,MotorType.kBrushless);
    CCMID = new CANcoder(12);

    controller = new XboxController(0);

    TMSE.getPIDController().setP(1);
    TMSE.getPIDController().setI(0);
    TMSE.getPIDController().setD(0);
    TMSE.getPIDController().setOutputRange(-1, 1);

    TMSD.getPIDController().setP(1);
    TMSD.getPIDController().setI(0);
    TMSD.getPIDController().setD(0);
    TMSD.getPIDController().setOutputRange(-1, 1);

    TMIE.getPIDController().setP(1);
    TMIE.getPIDController().setI(0);
    TMIE.getPIDController().setD(0);
    TMIE.getPIDController().setOutputRange(-1, 1);

    TMID.getPIDController().setP(1);
    TMID.getPIDController().setI(0);
    TMID.getPIDController().setD(0);
    TMID.getPIDController().setOutputRange(-1, 1);

    AMSE.getPIDController().setP(1);
    AMSE.getPIDController().setI(0);
    AMSE.getPIDController().setD(0);
    AMSE.getPIDController().setOutputRange(-0.3, 0.3);
    AMSE.getPIDController().setPositionPIDWrappingEnabled(true);
    AMSE.getPIDController().setPositionPIDWrappingMaxInput(360);
    AMSE.getPIDController().setPositionPIDWrappingMinInput(0);

    AMSD.getPIDController().setP(1);
    AMSD.getPIDController().setI(0);
    AMSD.getPIDController().setD(0);
    AMSD.getPIDController().setOutputRange(-0.3, 0.3);
    AMSD.getPIDController().setPositionPIDWrappingEnabled(true);
    AMSD.getPIDController().setPositionPIDWrappingMaxInput(360);
    AMSD.getPIDController().setPositionPIDWrappingMinInput(0);

    AMIE.getPIDController().setP(1);
    AMIE.getPIDController().setI(0);
    AMIE.getPIDController().setD(0);
    AMIE.getPIDController().setOutputRange(-0.3, 0.3);
    AMIE.getPIDController().setPositionPIDWrappingEnabled(true);
    AMIE.getPIDController().setPositionPIDWrappingMaxInput(360);
    AMIE.getPIDController().setPositionPIDWrappingMinInput(0);

    AMID.getPIDController().setP(1);
    AMID.getPIDController().setI(0);
    AMID.getPIDController().setD(0);
    AMID.getPIDController().setOutputRange(-0.3, 0.3);
    AMID.getPIDController().setPositionPIDWrappingEnabled(true);
    AMID.getPIDController().setPositionPIDWrappingMaxInput(360);
    AMID.getPIDController().setPositionPIDWrappingMinInput(0);

    TMSE.getEncoder().setPosition(0);
    TMSD.getEncoder().setPosition(0);
    TMIE.getEncoder().setPosition(0);
    TMID.getEncoder().setPosition(0);
    AMSE.getEncoder().setPosition(0);
    AMSD.getEncoder().setPosition(0);
    AMIE.getEncoder().setPosition(0);
    AMID.getEncoder().setPosition(0);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() 
  {
    





  }

  @Override
  public void teleopPeriodic() 
  {
    

    
    double joyLeft_rad = Math.atan2(controller.getRawAxis(0), controller.getRawAxis(1));
    double joyLeft_degrees = Math.toDegrees(joyLeft_rad);
    joyLeft_degrees = (joyLeft_degrees < 0) ? joyLeft_degrees + 360 : joyLeft_degrees;
  
    
    

    if (controller.getAButton()) 
    {
      pos_des = 300;
      //if (pos_des >= 360) pos_des = 0;
    }

    if (controller.getBButton()) 
    {
      pos_des = 20;
      //if (pos_des <= 0) pos_des = 360;
    }
    
    if (controller.getXButton()) 
    {
      pos_des = 0;
    }

    if (controller.getYButton() && trava_01) 
    {
      trava_01 = false;
      TG_PID = !TG_PID;
      if (TG_PID) 
      {
        AMID.getPIDController().setP(0);
        AMIE.getPIDController().setP(0);
        AMSD.getPIDController().setP(0);
        AMSE.getPIDController().setP(0);
      }
      else
      {
        AMID.getPIDController().setP(1);
        AMIE.getPIDController().setP(1);
        AMSD.getPIDController().setP(1);
        AMSE.getPIDController().setP(1);
      }
    }
    if (!controller.getYButton() && !trava_01) trava_01 = true;
    

    if (controller.getRawAxis(0) < -0.2 || controller.getRawAxis(0) > 0.2 || controller.getRawAxis(1) < -0.2 || controller.getRawAxis(1) > 0.2) 
    {
      AMID.getPIDController().setReference(joyLeft_degrees / 16.83816651075772, ControlType.kPosition);
      AMIE.getPIDController().setReference(joyLeft_degrees / 16.83816651075772, ControlType.kPosition);
      AMSD.getPIDController().setReference(joyLeft_degrees / 16.83816651075772, ControlType.kPosition);
      AMSE.getPIDController().setReference(joyLeft_degrees / 16.83816651075772, ControlType.kPosition);
    }
    else
    {
      AMID.getPIDController().setReference(0, ControlType.kPosition);
      AMIE.getPIDController().setReference(0, ControlType.kPosition);
      AMSD.getPIDController().setReference(0, ControlType.kPosition);
      AMSE.getPIDController().setReference(0, ControlType.kPosition);
    }
    


    SmartDashboard.putNumber("RPM AMSE", AMSE.getEncoder().getPosition());
    SmartDashboard.putNumber("RPM AMSD", AMSD.getEncoder().getPosition());
    SmartDashboard.putNumber("RPM AMIE", AMIE.getEncoder().getPosition());
    SmartDashboard.putNumber("RPM AMID", AMID.getEncoder().getPosition());
    SmartDashboard.putBoolean("BUT A", controller.getAButton());
    SmartDashboard.putNumber("Potencia AMSE", AMSE.getAppliedOutput());
    SmartDashboard.putNumber("Potencia AMSD", AMSD.getAppliedOutput());
    SmartDashboard.putNumber("Potencia AMIE", AMIE.getAppliedOutput());
    SmartDashboard.putNumber("Potencia AMID", AMID.getAppliedOutput());
    SmartDashboard.putNumber("Pos_des", pos_des);
    SmartDashboard.putNumber("Valor_Joy", joyLeft_degrees);
    SmartDashboard.putBoolean("TG_PID", TG_PID);


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
