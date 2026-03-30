// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

  private XboxController controle = new XboxController(0);

  private CANSparkMax shotter_itk = new CANSparkMax(23, MotorType.kBrushless);
  private CANSparkMax shotter_disp_sup = new CANSparkMax(21, MotorType.kBrushless);
  private CANSparkMax shotter_disp_inf = new CANSparkMax(22, MotorType.kBrushless);
  private CANSparkMax shotter_inc_right = new CANSparkMax(20, MotorType.kBrushless);
  private CANSparkMax shotter_inc_left = new CANSparkMax(24, MotorType.kBrushless);

  private DigitalInput sensor_shotterItk = new DigitalInput(0);

  SparkPIDController pidShutter = shotter_inc_right.getPIDController();


  // private int goSetPoint;
  private double setPoint;
  private int dispMode=0;
  private boolean shotter_startoff = false;
  private boolean shotter_descer = false;
  private boolean shotter_amp = false;

  private int engatilhado = 0;
  private int inProcess = 0;

  private boolean dispOkAmp=false;
  private double out_intake=0, out_shotter1=0, out_shotter2=0;
  @Override
  public void robotInit() 
  {
    pidShutter.setP(0.42);
    pidShutter.setI(0);
    pidShutter.setD(0);
    pidShutter.setIZone(0);
    pidShutter.setFF(0);
    pidShutter.setOutputRange(-0.1, 0.25);

    shotter_inc_left.setOpenLoopRampRate(1);
    shotter_inc_right.setOpenLoopRampRate(1);
    shotter_inc_left.follow(shotter_inc_right, true);

    shotter_inc_right.setIdleMode(IdleMode.kBrake);
    shotter_inc_right.setIdleMode(IdleMode.kBrake);
    shotter_itk.setIdleMode(IdleMode.kCoast);
    shotter_disp_sup.setInverted(false);

    shotter_inc_left.getEncoder().setPosition(0);
    shotter_inc_right.getEncoder().setPosition(0);
  }

  @Override
  public void robotPeriodic() 
  {
    SmartDashboard.putNumber("Encoder Shotter Inc L", shotter_inc_left.getEncoder().getPosition());
    SmartDashboard.putNumber("Encoder Shotter Inc R", shotter_inc_right.getEncoder().getPosition()); 
    SmartDashboard.putNumber("Encoder Shotter Itk", shotter_itk.getEncoder().getPosition());
    SmartDashboard.putBoolean("Sensor Shotik", sensor_shotterItk.get());
    SmartDashboard.putNumber("Engatilhado?", engatilhado);
    SmartDashboard.putNumber("Velocity Shotter Sup?", shotter_disp_sup.getEncoder().getVelocity());
    SmartDashboard.putNumber("Velocity Shotter Inf?", shotter_disp_inf.getEncoder().getVelocity());
    SmartDashboard.putNumber("Encoder Disp Sup", shotter_disp_sup.getEncoder().getPosition());
    SmartDashboard.putNumber("Encoder Disp Inf", shotter_disp_inf.getEncoder().getPosition());
  }

  @Override

  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() 
  {
    

    if(sensor_shotterItk.get()==true && inProcess==0 && engatilhado==0)
    {
      inProcess=1;
      engatilhado=1;
    }
    if(inProcess==1)
    {
      if(engatilhado==1)
      {
        out_intake=-0.6;
        shotter_itk.getEncoder().setPosition(0);
        if(sensor_shotterItk.get()==false)
        {
          engatilhado=2;
        }
      }

      if(engatilhado==2) // Avanço
      {
        int setAvancoNote=-1;
        if(shotter_itk.getEncoder().getPosition()>=(setAvancoNote))
        {
          out_intake=-0.6;
        }
        if(shotter_itk.getEncoder().getPosition()<(setAvancoNote))
        {
          shotter_itk.getEncoder().setPosition(0);
          engatilhado=3;
        }
      }
      if(engatilhado==3) // Recuo
      {
        double setRecuoNote=2.5;
        if(shotter_itk.getEncoder().getPosition()<=(setRecuoNote))
        {
          out_shotter1=-0.3;
          out_shotter2=out_shotter1;
          out_intake=1;
        }
        if(shotter_itk.getEncoder().getPosition()>(setRecuoNote))
        {
          shotter_itk.getEncoder().setPosition(0);
          engatilhado=4;
        }
      }
      if(engatilhado==4)
      {
        inProcess=0;
        engatilhado=5;
      }
    }

    if(engatilhado==5 && dispMode==1)
    {
      if(controle.getAButtonPressed())
      {
        shotter_startoff=!shotter_startoff;
      }
      if(shotter_startoff==true)
      {
        out_shotter1=1;
        out_shotter2=out_shotter1;
      }
      else
      {
        out_shotter1=0;
        out_shotter2=out_shotter1;
      }

      if(controle.getBButton()==true)
      {
        out_intake=-1;
        engatilhado=0;
      }
    }

    if(engatilhado==5 && controle.getLeftTriggerAxis()>0.2)
    {
      if(controle.getAButtonPressed())
      {
        shotter_startoff=!shotter_startoff;
      }
      if(shotter_startoff==true)
      {
        out_shotter1=0.1;
        out_shotter2=0.1;
        // if(shotter_disp_sup.getEncoder().getVelocity())
        // if(shotter_itk.getEncoder().getPosition()<-100)
        // {
        //   shotter_amp=false;
        //   engatilhado=0;
          
        // }
      }
      else
      {
        out_shotter1=0;
        out_shotter2=out_shotter1;
      }
      if(controle.getBButton()==true)
      {
        out_intake=-0.5;
        //engatilhado=0;
      }
    }

    if(controle.getRightBumperPressed())
    {
      pidShutter.setP(0.42);
      // pidShutter.setReference(2.4, ControlType.kPosition);
    }

    if(controle.getLeftBumperPressed())
    {
      pidShutter.setP(0.42);
      pidShutter.setReference(0, ControlType.kPosition);
    }
    if(controle.getBackButtonPressed())
    {
      shotter_descer=true;
      pidShutter.setReference(0, ControlType.kPosition);
    }

    if(shotter_inc_right.getEncoder().getPosition()<1 && shotter_descer==true)
    {
      pidShutter.setP(0);
      shotter_descer=false;
    }

    if(controle.getStartButton())
    {
      shotter_inc_left.getEncoder().setPosition(0);
      shotter_inc_right.getEncoder().setPosition(0);
    }


    if(controle.getYButtonPressed())
    {
      shotter_amp=true;
    }
      if(shotter_amp==true)
      {
          out_shotter1=0.4;
          out_shotter2=0;
          
          if(shotter_disp_inf.getEncoder().getVelocity()>1400 && dispOkAmp==false)
          {
            shotter_itk.getEncoder().setPosition(0);
            out_intake=-1;
            dispOkAmp=true;
          }
          if(shotter_itk.getEncoder().getPosition()<-100 && dispOkAmp==true)
          {
            out_shotter1=0;
            out_shotter2=0;
            out_intake=0;
            dispOkAmp=false;
            shotter_amp=false;
          }
      }
    // }

    shotter_itk.set(out_intake);
    shotter_disp_inf.set(out_shotter1);
    shotter_disp_sup.set(out_shotter2);
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
