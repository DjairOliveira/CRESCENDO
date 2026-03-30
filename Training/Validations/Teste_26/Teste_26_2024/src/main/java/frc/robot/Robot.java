// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


/*xbox*/
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType; 
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.Timer;

/** 
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class o
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  private XboxController Controle;
  
  private CANSparkMax motorNEO5, motorNEO6;

  boolean Tg=false, Ctr=false, Coletou=false, Coletar=false;

  int cont1=0, cont2=0, Mult=0;
  @Override
  public void robotInit() 
  {
    Controle = new XboxController(0);
    motorNEO5 = new CANSparkMax(5, MotorType.kBrushless); // Troque 1 pelo ID do Spark MAX conectado ao motor NEO1
    motorNEO6 = new CANSparkMax(6, MotorType.kBrushless); // Troque 2 pelo ID do Spark MAX conectado ao motor NEO2
  }
  
  @Override
  public void robotPeriodic() 
  {

  }

  @Override
  public void autonomousInit() 
  {

  }

  @Override
  public void autonomousPeriodic() 
  {

  }

  @Override
  public void teleopInit() 
  {
    Mult=0;
    cont2=0;
    Coletou=false;
  }

  @Override
  public void teleopPeriodic() 
  {
    double Corrente = motorNEO5.getOutputCurrent();

    if(Controle.getAButton()==true && Ctr==false)
    {
      Tg=!Tg;
      Ctr=true;
    }
    if(Controle.getAButton()==false && Ctr==true)
    {
      Ctr=false;
    }

    if(Tg==true)
    {
      if(Coletou==false)
      {
        Mult=1;
        cont2=0;
        Coletar=true;
        if(cont1<=60) cont1++;
      }
      if(cont1>60 && Corrente>=14)
      {
        Mult=0;
        Coletar=true;
        Coletou=true;
      }
    }
    if(Tg==false && Coletou==true)
    {
      cont1=0;
      cont2=0;
      Mult=(-1);
      Coletou=false;
    }

    if(cont2<=100 && Mult==(-1))
    {
      cont2++;
    }
    if(cont2>100) 
    {
      Mult=0;
    }

    motorNEO6.set(-0.5*Mult);
    motorNEO5.set(-0.5*Mult);

    SmartDashboard.putNumber("Corrente", Corrente);
    SmartDashboard.putBoolean("Coletar", Coletar);
    SmartDashboard.putBoolean("Coletou", Coletou);
    SmartDashboard.putBoolean("TG", Tg);
    SmartDashboard.putNumber("Cont1", cont1);
    SmartDashboard.putNumber("Cont2", cont2);
    SmartDashboard.putNumber("Mult", Mult);
  }

    @Override
  public void disabledInit() 
  {
  }

  @Override
  public void disabledPeriodic() 
  {
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
