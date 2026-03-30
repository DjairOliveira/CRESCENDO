// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenixpro.StatusSignalValue;
//import com.ctre.phoenixpro.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import javax.management.timer.TimerMBean;

import com.ctre.phoenix.sensors.CANCoder;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
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
  private boolean FLAG1_PID_P = false;
  private boolean FLAG2_PID_P = false;

  private boolean FLAG_SET1=false;
  private boolean FLAG_SET2=false;
  private boolean Invert=false;
  private boolean trava = false;
  private boolean trava2 = false;
  private boolean trava3 = false;

  public static boolean InRange(double valor, double valorMinimo, double valorMaximo)
  {
    return valor >= valorMinimo && valor <= valorMaximo;
  }
  // Função para normalizar um ângulo para o intervalo [0, 360)
  public double normalizarAngulo (double angulo)
  {
    angulo = angulo % 360;
    if (angulo < 0) angulo += 360;
    return angulo;
  }
    // Função para calcular o menor caminho entre dois ângulos
  public double calcularMenorCaminho(double anguloAtual, double novoAngulo)
  {
    // Normalizar os ângulos para o intervalo [0, 360)
    anguloAtual = normalizarAngulo(anguloAtual);
    novoAngulo = normalizarAngulo(novoAngulo);
    // Calcular a diferença entre os ângulos
    double diferenca = novoAngulo - anguloAtual;
    // Verificar se o caminho mais curto é através de 0 ou 360
    if (Math.abs(diferenca) > 180)
    {
      if (diferenca > 0)
      {
        diferenca -= 360;
      }
      else 
      {
        diferenca += 360;
      }
    }
    return diferenca;
  }

  @Override
  public void robotInit() 
  {
    TMSE = new CANSparkMax(5,MotorType.kBrushless);
    AMSE = new CANSparkMax(6,MotorType.kBrushless);
    CCMSE = new CANCoder(9);

    TMSD = new CANSparkMax(7,MotorType.kBrushless);
    AMSD = new CANSparkMax(8,MotorType.kBrushless);
    CCMSD = new CANCoder(10);

    TMIE = new CANSparkMax(3,MotorType.kBrushless);
    AMIE = new CANSparkMax(4,MotorType.kBrushless);
    CCMIE = new CANCoder(11);

    TMID = new CANSparkMax(1,MotorType.kBrushless);
    AMID = new CANSparkMax(2,MotorType.kBrushless);
    CCMID = new CANCoder(12);

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
    AMSE.getPIDController().setPositionPIDWrappingMaxInput(0);
    AMSE.getPIDController().setPositionPIDWrappingMinInput(21.38);

    AMSD.getPIDController().setP(1);
    AMSD.getPIDController().setI(0);
    AMSD.getPIDController().setD(0);
    AMSD.getPIDController().setOutputRange(-0.3, 0.3);
    AMSD.getPIDController().setPositionPIDWrappingEnabled(true);
    AMSD.getPIDController().setPositionPIDWrappingMaxInput(0);
    AMSD.getPIDController().setPositionPIDWrappingMinInput(21.38);

    AMIE.getPIDController().setP(1);
    AMIE.getPIDController().setI(0);
    AMIE.getPIDController().setD(0);
    AMIE.getPIDController().setOutputRange(-0.3, 0.3);
    AMIE.getPIDController().setPositionPIDWrappingEnabled(true);
    AMIE.getPIDController().setPositionPIDWrappingMaxInput(0);
    AMIE.getPIDController().setPositionPIDWrappingMinInput(21.38);

    AMID.getPIDController().setP(1);
    AMID.getPIDController().setI(0);
    AMID.getPIDController().setD(0);
    AMID.getPIDController().setOutputRange(-0.3, 0.3);
    AMID.getPIDController().setPositionPIDWrappingEnabled(true);
    AMID.getPIDController().setPositionPIDWrappingMaxInput(0);
    AMID.getPIDController().setPositionPIDWrappingMinInput(21.38);

    TMSE.getEncoder().setPosition(0);
    TMSD.getEncoder().setPosition(0);
    TMIE.getEncoder().setPosition(0);
    TMID.getEncoder().setPosition(0);
    AMSE.getEncoder().setPosition(0);
    AMSD.getEncoder().setPosition(0);
    AMIE.getEncoder().setPosition(0);
    AMID.getEncoder().setPosition(0);

    // AMSE.getEncoder().setPosition(CCMSE.getAbsolutePosition() / 16.83816651075772);
    // AMSD.getEncoder().setPosition(CCMSD.getAbsolutePosition() / 16.83816651075772);
    // AMIE.getEncoder().setPosition(CCMIE.getAbsolutePosition() / 16.83816651075772);
    // AMID.getEncoder().setPosition(CCMID.getAbsolutePosition() / 16.83816651075772);
    
    // AMID.getPIDController().setP(0.2); 
    // AMIE.getPIDController().setP(0.2);
    // AMSD.getPIDController().setP(0.2);
    // AMSE.getPIDController().setP(0.2);

    // AMID.getPIDController().setReference(276.41 / 16.83816651075772, ControlType.kPosition);  
    // AMIE.getPIDController().setReference(324.35 / 16.83816651075772, ControlType.kPosition); 
    // AMSD.getPIDController().setReference(148.88 / 16.83816651075772, ControlType.kPosition); 
    // AMSE.getPIDController().setReference(174.5 / 16.83816651075772, ControlType.kPosition);
  }

  @Override
  public void robotPeriodic() 
  {
    SmartDashboard.putNumber("CANCODER_SD", CCMSD.getAbsolutePosition());
    SmartDashboard.putNumber("CANCODER_SE", CCMSE.getAbsolutePosition());
    SmartDashboard.putNumber("CANCODER_IE", CCMIE.getAbsolutePosition());
    SmartDashboard.putNumber("CANCODER_ID", CCMID.getAbsolutePosition());
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() 
  {
    FLAG_SET1=false;
    FLAG_SET2=false;

    // double InitCANSD = 148.88-CCMSD.getAbsolutePosition();
    // double InitCANSE = 174.5-CCMSE.getAbsolutePosition();
    // double InitCANIE = 324.35-CCMIE.getAbsolutePosition();
    // double InitCANID = 276.41-CCMID.getAbsolutePosition();

    // AMSE.getEncoder().setPosition(CCMSE.getAbsolutePosition() / 16.83816651075772);
    // AMSD.getEncoder().setPosition(CCMSD.getAbsolutePosition() / 16.83816651075772);
    // AMIE.getEncoder().setPosition(CCMIE.getAbsolutePosition() / 16.83816651075772);
    // AMID.getEncoder().setPosition(CCMID.getAbsolutePosition() / 16.83816651075772);
    
    // AMID.getPIDController().setP(0.5); 
    // AMIE.getPIDController().setP(0.5);
    // AMSD.getPIDController().setP(0.5);
    // AMSE.getPIDController().setP(0.5);

    // AMID.getPIDController().setReference(276.41 / 16.83816651075772, ControlType.kPosition);  
    // AMIE.getPIDController().setReference(324.35 / 16.83816651075772, ControlType.kPosition); 
    // AMSD.getPIDController().setReference(148.88 / 16.83816651075772, ControlType.kPosition); 
    // AMSE.getPIDController().setReference(174.5 / 16.83816651075772, ControlType.kPosition);

    // if(AMSE.getAppliedOutput() == 0 && AMSD.getAppliedOutput())

    // AMIE.getAppliedOutput();
    // AMID.getAppliedOutput();
    // while(AMIE.getEncoder().getPosition()/0.0593888888888889)
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
  
    JoyLeft_degrees*=(-1);
    JoyLeft_degrees = (JoyLeft_degrees < 0) ? JoyLeft_degrees + 360 : JoyLeft_degrees;

    if((AMSE.getAppliedOutput()>-0.1 && AMSE.getAppliedOutput()<0.1) && trava3==false)
    {
      trava3=true;
    }
    else
    {
    if (controller.getRawAxis(4) <= -0.15 || controller.getRawAxis(4) >= 0.15 || controller.getRawAxis(5) <= -0.15 || controller.getRawAxis(5) >= 0.15) 
    {
      trava2=false;
      trava = true;
      if(FLAG1_PID_P==false)
      {
        AMID.getPIDController().setP(0.2); 
        AMIE.getPIDController().setP(0.2);
        AMSD.getPIDController().setP(0.2);
        AMSE.getPIDController().setP(0.2);

        AMID.getPIDController().setReference(45 / 16.83816651075772, ControlType.kPosition);  // 225
        AMIE.getPIDController().setReference(135 / 16.83816651075772, ControlType.kPosition); // 315
        AMSD.getPIDController().setReference(315 / 16.83816651075772, ControlType.kPosition); // 135
        AMSE.getPIDController().setReference(225 / 16.83816651075772, ControlType.kPosition); // 45

        FLAG1_PID_P=true;
      }

      Velocit = (controller.getRawAxis(4)/4);
    }
    else
    {
      if (((controller.getRawAxis(0) <= -0.2 || controller.getRawAxis(0) >= 0.2 || controller.getRawAxis(1) <= -0.2 || controller.getRawAxis(1) >= 0.2))) 
      {
        /*
         * 2° - Atribuir Range para ajuste do P, de forma que ele ajuste com o ganho de P=1 quando a posição desejado estiver longe da meta
         *  e um ganho de P=0.2 quando a posição desejado estiver próximo da meta
        */
        if(FLAG2_PID_P==false)
        {
          AMID.getPIDController().setP(0.2); 
          AMIE.getPIDController().setP(0.2);
          AMSD.getPIDController().setP(0.2);
          AMSE.getPIDController().setP(0.2);
  
          FLAG2_PID_P=true;
        }
  
        FLAG1_PID_P=false;
  
        AMID.getPIDController().setReference(JoyLeft_degrees / 16.83816651075772, ControlType.kPosition);
        AMIE.getPIDController().setReference(JoyLeft_degrees / 16.83816651075772, ControlType.kPosition);
        AMSD.getPIDController().setReference(JoyLeft_degrees / 16.83816651075772, ControlType.kPosition);
        AMSE.getPIDController().setReference(JoyLeft_degrees / 16.83816651075772, ControlType.kPosition);
      }
      else
      {
        if(FLAG1_PID_P==true || FLAG2_PID_P==true)
        {
          AMID.getPIDController().setP(0);
          AMIE.getPIDController().setP(0);
          AMSD.getPIDController().setP(0);
          AMSE.getPIDController().setP(0);

          FLAG2_PID_P=false;
        }
      }
      FLAG1_PID_P=false;
    }

    if(controller.getAButton()==true && trava2==false)
    {
      AMID.getPIDController().setP(0.2); 
      AMIE.getPIDController().setP(0.2);
      AMSD.getPIDController().setP(0.2);
      AMSE.getPIDController().setP(0.2);

      AMID.getPIDController().setReference(0, ControlType.kPosition);
      AMIE.getPIDController().setReference(0, ControlType.kPosition);
      AMSD.getPIDController().setReference(0, ControlType.kPosition);
      AMSE.getPIDController().setReference(0, ControlType.kPosition);
      trava2=true;
    }
  }
    SmartDashboard.putNumber("RPM AMSE", AMSE.getEncoder().getPosition());
    SmartDashboard.putNumber("RPM AMSD", AMSD.getEncoder().getPosition());
    SmartDashboard.putNumber("RPM AMIE", AMIE.getEncoder().getPosition());
    SmartDashboard.putNumber("RPM AMID", AMID.getEncoder().getPosition());
    SmartDashboard.putBoolean("BTN A", controller.getAButton());

    SmartDashboard.putNumber("CONV° AMIE", AMIE.getEncoder().getPosition()/0.0593888888888889);

    SmartDashboard.putNumber("Potencia AMSE", AMSE.getAppliedOutput());
    SmartDashboard.putNumber("Potencia AMSD", AMSD.getAppliedOutput());
    SmartDashboard.putNumber("Potencia AMIE", AMIE.getAppliedOutput());
    SmartDashboard.putNumber("Potencia AMID", AMID.getAppliedOutput());
    SmartDashboard.putNumber("Valor_Joy°", JoyLeft_degrees);
    SmartDashboard.putNumber("Valor_Joy2°", JoyRight_degrees);
    SmartDashboard.putBoolean("trava", trava);

    TMSE.set(Velocit);
    TMSD.set(Velocit);
    TMIE.set(Velocit);
    TMID.set(Velocit);
  }
  
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() 
  {
    // AMSE.getEncoder().setPosition(0);
    // AMSD.getEncoder().setPosition(0);
    // AMIE.getEncoder().setPosition(0);
    // AMID.getEncoder().setPosition(0);
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
