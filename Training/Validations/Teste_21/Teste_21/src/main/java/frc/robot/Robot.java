// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenixpro.StatusSignalValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import javax.management.timer.TimerMBean;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;

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
  private CANSparkMax TMSE;
  private CANSparkMax TMSD;
  private CANSparkMax TMIE;
  private CANSparkMax TMID;

  private CANSparkMax AMSE;
  private CANSparkMax AMSD;
  private CANSparkMax AMIE;
  private CANSparkMax AMID;

  private CANCoder CCMSE;
  private CANCoder CCMSD;
  private CANCoder CCMIE;
  private CANCoder CCMID;

  private XboxController controller;

  private Pigeon2 Pigeon;

  private boolean TOG = false;
  private double CONT = 0;

  private boolean FLAG1_PID_P = false;
  private boolean FLAG2_PID_P = false;

  private boolean FLAG_SET1=false;
  private boolean FLAG_SET2=false;
  private boolean Invert=false;
  private boolean trava = false;
  private boolean trava2 = false;
  private boolean trava3 = false;
  @Override
  public void robotInit() 
  {
    controller = new XboxController(0);
    Pigeon = new Pigeon2(13);
    
    TMIE = new CANSparkMax(3,MotorType.kBrushless);
    AMIE = new CANSparkMax(4,MotorType.kBrushless);
    AMIE.setInverted(true);
    CCMIE = new CANCoder(9);

    TMID = new CANSparkMax(1,MotorType.kBrushless);
    AMID = new CANSparkMax(2,MotorType.kBrushless);
    AMID.setInverted(true);
    CCMID = new CANCoder(12);

    TMSE = new CANSparkMax(5,MotorType.kBrushless);
    AMSE = new CANSparkMax(6,MotorType.kBrushless);
    AMSE.setInverted(true);
    CCMSE = new CANCoder(10);

    TMSD = new CANSparkMax(7,MotorType.kBrushless);
    AMSD = new CANSparkMax(8,MotorType.kBrushless);
    AMSD.setInverted(true);
    CCMSD = new CANCoder(11);

    TMIE.getPIDController().setP(1);
    TMIE.getPIDController().setI(0);
    TMIE.getPIDController().setD(0);
    TMIE.getPIDController().setOutputRange(-1, 1);

    AMIE.getPIDController().setP(0.2);
    AMIE.getPIDController().setI(0);
    AMIE.getPIDController().setD(0);
    AMIE.getPIDController().setOutputRange(-0.3, 0.3);
    AMIE.getPIDController().setPositionPIDWrappingEnabled(true);
    AMIE.getPIDController().setPositionPIDWrappingMaxInput(21.38);
    AMIE.getPIDController().setPositionPIDWrappingMinInput(0);

    TMID.getPIDController().setP(1);
    TMID.getPIDController().setI(0);
    TMID.getPIDController().setD(0);
    TMID.getPIDController().setOutputRange(-1, 1);

    AMID.getPIDController().setP(0.2);
    AMID.getPIDController().setI(0);
    AMID.getPIDController().setD(0);
    AMID.getPIDController().setOutputRange(-0.3, 0.3);
    AMID.getPIDController().setPositionPIDWrappingEnabled(true);
    AMID.getPIDController().setPositionPIDWrappingMaxInput(21.38);
    AMID.getPIDController().setPositionPIDWrappingMinInput(0);

    TMSE.getPIDController().setP(1);
    TMSE.getPIDController().setI(0);
    TMSE.getPIDController().setD(0);
    TMSE.getPIDController().setOutputRange(-1, 1);

    AMSE.getPIDController().setP(0.2);
    AMSE.getPIDController().setI(0);
    AMSE.getPIDController().setD(0);
    AMSE.getPIDController().setOutputRange(-0.3, 0.3);
    AMSE.getPIDController().setPositionPIDWrappingEnabled(true);
    AMSE.getPIDController().setPositionPIDWrappingMaxInput(21.38);
    AMSE.getPIDController().setPositionPIDWrappingMinInput(0);

    TMSD.getPIDController().setP(1);
    TMSD.getPIDController().setI(0);
    TMSD.getPIDController().setD(0);
    TMSD.getPIDController().setOutputRange(-1, 1);

    AMSD.getPIDController().setP(0.2);
    AMSD.getPIDController().setI(0);
    AMSD.getPIDController().setD(0);
    AMSD.getPIDController().setOutputRange(-0.3, 0.3);
    AMSD.getPIDController().setPositionPIDWrappingEnabled(true);
    AMSD.getPIDController().setPositionPIDWrappingMaxInput(21.38);
    AMSD.getPIDController().setPositionPIDWrappingMinInput(0);

    TMIE.getEncoder().setPosition(0);
    AMIE.getEncoder().setPosition(0);

    TMID.getEncoder().setPosition(0);
    AMID.getEncoder().setPosition(0);

    TMSE.getEncoder().setPosition(0);
    AMSE.getEncoder().setPosition(0);

    TMSD.getEncoder().setPosition(0);
    AMSD.getEncoder().setPosition(0);
    // AMSE.getEncoder().setPosition(CCMSE.getAbsolutePosition()/ 16.83816651075772);
    Pigeon.setYaw(0);

  }

  @Override
  public void robotPeriodic() 
  {
    SmartDashboard.putNumber("CANCODER_SE", CCMID.getAbsolutePosition());
    SmartDashboard.putNumber("POSI DO ENCODER", AMID.getEncoder().getPosition());
    SmartDashboard.putNumber("ENCODER CANCODER", AMID.getEncoder().getPosition()/0.0593888888888889);
    SmartDashboard.putBoolean("TOGLE", TOG);
    SmartDashboard.putNumber("CONTADOR", CONT);
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() 
  {
    FLAG1_PID_P=false;
    FLAG2_PID_P=false;
  }

  @Override
  public void teleopPeriodic() 
  {
    double GatRight = controller.getRawAxis(3);
    double Velocit = GatRight;

    double JoyLeft_rad = Math.atan2(controller.getRawAxis(0), controller.getRawAxis(1));
    double JoyLeft_degrees = Math.toDegrees(JoyLeft_rad);

    double JoyRight_rad = Math.atan2(controller.getRawAxis(4), controller.getRawAxis(5));
    double JoyRight_degrees = Math.toDegrees(JoyRight_rad);

    double[] YPR = new double[3];

    Pigeon.getYawPitchRoll(YPR);

    double Yaw = YPR[0];
    double Pitch = YPR[1];
    double Roll = YPR[2];
    Yaw*=(-1);
    double absoluteYaw = Yaw % 360;
    if (absoluteYaw<0) absoluteYaw += 360; // Lógica de retorno ao cruzar os 360° ou decrementar de 0°


    if (controller.getRawAxis(4) <= -0.15 || controller.getRawAxis(4) >= 0.15 || controller.getRawAxis(5) <= -0.15 || controller.getRawAxis(5) >= 0.15) 
    {
      if(FLAG1_PID_P==false)
      {
        AMID.getPIDController().setP(0.2); 
        AMIE.getPIDController().setP(0.2);
        AMSD.getPIDController().setP(0.2);
        AMSE.getPIDController().setP(0.2);

        AMID.getPIDController().setReference(315 / 16.83816651075772, ControlType.kPosition);
        AMIE.getPIDController().setReference(225 / 16.83816651075772, ControlType.kPosition);
        AMSD.getPIDController().setReference(45 / 16.83816651075772, ControlType.kPosition);
        AMSE.getPIDController().setReference(115 / 16.83816651075772, ControlType.kPosition);

        FLAG1_PID_P=true;
      }

      Velocit = (controller.getRawAxis(4)/4);
    }
    else
    {
      FLAG1_PID_P=false;
    }
    if(FLAG1_PID_P==false)
    {
      if (controller.getRawAxis(0) <= -0.15 || controller.getRawAxis(0) >= 0.15 || controller.getRawAxis(1) <= -0.15 || controller.getRawAxis(1) >= 0.15) 
      {
        if(FLAG2_PID_P==false)
        {
          AMIE.getPIDController().setP(0.2);
          AMID.getPIDController().setP(0.2);
          AMSE.getPIDController().setP(0.2);
          AMSD.getPIDController().setP(0.2);
          FLAG2_PID_P=true;
        }
        AMIE.getPIDController().setReference((absoluteYaw+JoyLeft_degrees) / 16.83816651075772, ControlType.kPosition);
        AMID.getPIDController().setReference((absoluteYaw+JoyLeft_degrees) / 16.83816651075772, ControlType.kPosition);
        AMSE.getPIDController().setReference((absoluteYaw+JoyLeft_degrees) / 16.83816651075772, ControlType.kPosition);
        AMSD.getPIDController().setReference((absoluteYaw+JoyLeft_degrees) / 16.83816651075772, ControlType.kPosition);
      }
      else
      {
        if(controller.getAButton()==false)
        {
          AMIE.getPIDController().setP(0);
          AMID.getPIDController().setP(0);
          AMSE.getPIDController().setP(0);
          AMSD.getPIDController().setP(0);
        }
        FLAG2_PID_P=false;
      }
    }

    if(controller.getAButton()==true)
    {
      AMIE.getPIDController().setP(0.2);
      AMID.getPIDController().setP(0.2);
      AMSE.getPIDController().setP(0.2);
      AMSD.getPIDController().setP(0.2);

      AMIE.getPIDController().setReference(0, ControlType.kPosition);
      AMID.getPIDController().setReference(0, ControlType.kPosition);
      AMSE.getPIDController().setReference(0, ControlType.kPosition);
      AMSD.getPIDController().setReference(0, ControlType.kPosition);
    }

    SmartDashboard.putBoolean("BTN A", controller.getAButton());
    SmartDashboard.putNumber("PIGEON Yaw", Yaw);
    SmartDashboard.putNumber("PIGEON Pitch", Pitch);
    SmartDashboard.putNumber("PIGEON Roll", Roll);
    SmartDashboard.putNumber("Valor_Joy°", JoyLeft_degrees);
    SmartDashboard.putNumber("yaw abs", absoluteYaw);

    TMIE.set(Velocit);
    TMID.set(Velocit);
    TMSE.set(Velocit);
    TMSD.set(Velocit);
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
