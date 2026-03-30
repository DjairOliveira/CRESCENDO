// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Código: Swerve Drive V1.9
 * Equipe: 9168 AGROBOT
 * Desenvolvido por: Djair Oliveira & João Pistore.
 * 
 * As bibliotecas utilizadas e as versões dos softwares são baseadas na versão 2023.
 * 
 * O código em questão foi realizado em 2 meses e meio, e contou com o auxílio do Luca Carvalho
 * da equipe 1156 Under Control, no esclarecimento de algumas dúvidas e do chatGPT para o desenvolvimento de algumas funções.
 * 
 * O código ainda possui alguns erros de locomoção, o desenvolvimento do código não foi realizado envolvendo
 * calculos matématicos muito complexos, como por exemplo calculos relacionados a velocidade angular.
 */
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.Pigeon2;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private CANSparkMax TMSE;                         // TRAÇÃO MOTOR SUPERIOR ESQUERDO
  private CANSparkMax TMSD;                         // TRAÇÃO MOTOR SUPERIOR DIREITO
  private CANSparkMax TMIE;                         // TRAÇÃO MOTOR INFERIOR ESQUERDO
  private CANSparkMax TMID;                         // TRAÇÃO MOTOR INFERIOR DIREITO

  private CANSparkMax AMSE;                         // ANGULO MOTOR SUPERIOR ESQUERDO
  private CANSparkMax AMSD;                         // ANGULO MOTOR SUPERIOR DIREITO
  private CANSparkMax AMIE;                         // ANGULO MOTOR INFERIOR ESQUERDO
  private CANSparkMax AMID;                         // ANGULO MOTOR INFERIOR DIREITO

  private static XboxController controller;

  private Pigeon2 Pigeon;

  private double Ref_Encoder = 16.83816651075772;     // Referência do Encoder: 360°/21.38 = 16.83816651075772 (21.38 = 360° do Encoder)
  private double Ref_PID_P = 0.3;                     // Referência Proporcional PID: Valor de SetPoint da Proporcional PID
  private double Ref_Min_Max_Wrapping = 0.3;         // Range Máximo e Minimo PID Wrapping: Valor minimo e máximo que vai sair para o motor

  private boolean[] Var_Ctr = new boolean[2];         // Variaveis de controle

  public static boolean Analog_Stimulated(int Axisx, int Axisy, double Range)   // Verifica se o Joystick saiu da posição inicial
  {
    return (controller.getRawAxis(Axisx) <= (-Range) || controller.getRawAxis(Axisx) >= Range || controller.getRawAxis(Axisy) <= (-Range) || controller.getRawAxis(Axisy) >= Range);
  }

  public static void SPARKMAX_CONFIG_TRACAO(CANSparkMax Motor)                        // Configura o PID e range de saida para o motor Alvo
  {
    Motor.getPIDController().setP(1);
    Motor.getPIDController().setI(0);
    Motor.getPIDController().setD(0);
    Motor.getPIDController().setOutputRange(-1, 1);
  }

  public static void SPARKMAX_CONFIG_ANGULO (CANSparkMax Motor, double Min_Max_Wpg)   // Configura o PID, o range de saida e o Modo Wrapping
  {
    Motor.getPIDController().setP(0.2);
    Motor.getPIDController().setI(0);
    Motor.getPIDController().setD(0);
    Motor.getPIDController().setOutputRange(Min_Max_Wpg*(-1), Min_Max_Wpg);
    Motor.getPIDController().setPositionPIDWrappingEnabled(true);
    Motor.getPIDController().setPositionPIDWrappingMaxInput(21.38);
    Motor.getPIDController().setPositionPIDWrappingMinInput(0);
  }

  public static void SET_ALL_PID_P(CANSparkMax Inf_Esq, CANSparkMax Inf_Dir, CANSparkMax Sup_Esq, CANSparkMax Sup_Dir, double PID_P)  // Configura o P de todos os motores
  {
    Inf_Esq.getPIDController().setP(PID_P);
    Inf_Dir.getPIDController().setP(PID_P);
    Sup_Esq.getPIDController().setP(PID_P);
    Sup_Dir.getPIDController().setP(PID_P);
  }

  @Override
  public void robotInit() 
  {
    controller = new XboxController(0);
    Pigeon = new Pigeon2(13);
    
    TMIE = new CANSparkMax(7,MotorType.kBrushless);
    AMIE = new CANSparkMax(8,MotorType.kBrushless);
    AMIE.setInverted(true);

    TMID = new CANSparkMax(5,MotorType.kBrushless);
    AMID = new CANSparkMax(6,MotorType.kBrushless);
    AMID.setInverted(true);

    TMSE = new CANSparkMax(1,MotorType.kBrushless);
    AMSE = new CANSparkMax(2,MotorType.kBrushless);
    AMSE.setInverted(true);

    TMSD = new CANSparkMax(3,MotorType.kBrushless);
    AMSD = new CANSparkMax(4,MotorType.kBrushless);
    AMSD.setInverted(true);

    SPARKMAX_CONFIG_TRACAO(TMIE);
    SPARKMAX_CONFIG_TRACAO(TMID);
    SPARKMAX_CONFIG_TRACAO(TMSD);
    SPARKMAX_CONFIG_TRACAO(TMSE);

    SPARKMAX_CONFIG_ANGULO(AMIE, Ref_Min_Max_Wrapping);
    SPARKMAX_CONFIG_ANGULO(AMID, Ref_Min_Max_Wrapping);
    SPARKMAX_CONFIG_ANGULO(AMSD, Ref_Min_Max_Wrapping);
    SPARKMAX_CONFIG_ANGULO(AMSE, Ref_Min_Max_Wrapping);

    TMIE.getEncoder().setPosition(0);
    AMIE.getEncoder().setPosition(0);

    TMID.getEncoder().setPosition(0);
    AMID.getEncoder().setPosition(0);

    TMSE.getEncoder().setPosition(0);
    AMSE.getEncoder().setPosition(0);

    TMSD.getEncoder().setPosition(0);
    AMSD.getEncoder().setPosition(0);

    Pigeon.setYaw(0);
  }

  @Override
  public void robotPeriodic() 
  {

  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() 
  {
    Var_Ctr[0]=false;
    Var_Ctr[1]=false;

    AMIE.setIdleMode(IdleMode.kBrake);
    TMIE.setIdleMode(IdleMode.kBrake);
    AMSE.setIdleMode(IdleMode.kBrake);
    TMSE.setIdleMode(IdleMode.kBrake);
    AMSD.setIdleMode(IdleMode.kBrake);
    TMSD.setIdleMode(IdleMode.kBrake);
    AMID.setIdleMode(IdleMode.kBrake);
    TMID.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void teleopPeriodic() 
  {
    double MTSD=0;    // Motor Tração Superior Direito
    double MTID=0;    // Motor Tração Inferior Direito
    double MTSE=0;    // Motor Tração Superior Esquerdo
    double MTIE=0;    // Motor Tração Inferior Esquerdo

    double MAIE=0;   // Motor Tração Inferior Esquerdo
    double MASE=0;   // Motor Tração Superior Esquerdo
    double MAID=0;   // Motor Tração Inferior Direito
    double MASD=0;   // Motor Tração Superior Direito

    double Velocidade = controller.getRawAxis(3);

    double JoyLeft_degrees = Math.toDegrees(Math.atan2(controller.getRawAxis(0), controller.getRawAxis(1)));
    double JoyRight_degrees = Math.toDegrees(Math.atan2(controller.getRawAxis(4), controller.getRawAxis(5)));

    double Yaw    = Pigeon.getYaw().getValueAsDouble();
    double Pitch  = Pigeon.getPitch().getValueAsDouble();
    double Roll   = Pigeon.getRoll().getValueAsDouble();

    Yaw*=(-1);
    double absoluteYaw = Yaw % 360;

    if (absoluteYaw<0) absoluteYaw += 360;          // Lógica de retorno ao cruzar os 360° ou decrementar de 0°

    if ((Analog_Stimulated(4, 5, 0.15)==true) && (Analog_Stimulated(0, 1, 0.15)==false)) 
    {
      if(Var_Ctr[0]==false)
      {
        AMID.getPIDController().setP(Ref_PID_P);
        AMIE.getPIDController().setP(Ref_PID_P);
        AMSD.getPIDController().setP(Ref_PID_P);
        AMSE.getPIDController().setP(Ref_PID_P);

        AMID.getPIDController().setReference(315 / Ref_Encoder, ControlType.kPosition);
        AMIE.getPIDController().setReference(225 / Ref_Encoder, ControlType.kPosition);
        AMSD.getPIDController().setReference(45 / Ref_Encoder, ControlType.kPosition);
        AMSE.getPIDController().setReference(115 / Ref_Encoder, ControlType.kPosition);

        Var_Ctr[0]=true;
      }
      Velocidade = (controller.getRawAxis(4)/4);
      MTSD = Velocidade;
      MTID = Velocidade;
      MTIE = Velocidade;
      MTSE = Velocidade;
    }
    else
    {
      Var_Ctr[0]=false;
    }

    if(Var_Ctr[0]==false)
    {
      if (Analog_Stimulated(0, 1, 0.15)) 
      {
        if(Var_Ctr[1]==false)
        {
          SET_ALL_PID_P(AMIE, AMID, AMSE, AMSD, Ref_PID_P);
          Var_Ctr[1]=true;
        }

        MAIE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder;
        MAID = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder;
        MASE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder;
        MASD = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder;

        if(controller.getRawAxis(4) >= 0.15 || controller.getRawAxis(4) <= -0.15 )
        {
          if((JoyRight_degrees>=135 || JoyRight_degrees<=-134.999) && (JoyLeft_degrees>=135 || JoyLeft_degrees<=-134.999))
          {
            if(absoluteYaw>=315 || absoluteYaw<=44.999)
            {
              MASE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder;
              MASD = (absoluteYaw+JoyRight_degrees) / Ref_Encoder;
              MAID = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder;
              MAIE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder;
            }
        
            if(absoluteYaw>=45 && absoluteYaw<=134.999)
            {
              MAIE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder;
              MASE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder;
              MASD = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder;
              MAID = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder;
            }
  
            if(absoluteYaw>=135 && absoluteYaw<=224.999)
            {
              MAIE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder;
              MAID = (absoluteYaw+JoyRight_degrees) / Ref_Encoder;
              MASD = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder;
              MASE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder;
            }
            if(absoluteYaw>=225 && absoluteYaw<=314.999)
            {
              MASD = (absoluteYaw+JoyRight_degrees) / Ref_Encoder;
              MAID = (absoluteYaw+JoyRight_degrees) / Ref_Encoder;
              MASE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder;
              MAIE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder;
            }
          }
          if((JoyRight_degrees>=(-135) && JoyRight_degrees<=(-44.999)) && (JoyLeft_degrees>=(-135) && JoyLeft_degrees<=(-44.999)))
          {
            if(absoluteYaw>=315 || absoluteYaw<=44.999)
            {
              MASE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder;
              MAIE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder;
              MAID = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder;
              MASD = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder;
            }
        
            if(absoluteYaw>=45 && absoluteYaw<=134.999)
            {
              MAIE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder;
              MAID = (absoluteYaw+JoyRight_degrees) / Ref_Encoder;
              MASD = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder;
              MASE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder;
            }
  
            if(absoluteYaw>=135 && absoluteYaw<=224.999)
            {
              MASD = (absoluteYaw+JoyRight_degrees) / Ref_Encoder;
              MAID = (absoluteYaw+JoyRight_degrees) / Ref_Encoder;
              MASE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder;
              MAIE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder;
            }
            if(absoluteYaw>=225 && absoluteYaw<=314.999)
            {
              MASD = (absoluteYaw+JoyRight_degrees) / Ref_Encoder;
              MASE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder;
              MAIE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder;
              MAID = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder;
            }
          }
          if((JoyRight_degrees>=45 && JoyRight_degrees<=134.999) && (JoyLeft_degrees>=45 && JoyLeft_degrees<=134.999))
          {
            if(absoluteYaw>=315 || absoluteYaw<=44.999)
            {
              MASD = (absoluteYaw+JoyRight_degrees) / Ref_Encoder;
              MAID = (absoluteYaw+JoyRight_degrees) / Ref_Encoder;
              MAIE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder;
              MASE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder;
            }
        
            if(absoluteYaw>=45 && absoluteYaw<=134.999)
            {
              MASD = (absoluteYaw+JoyRight_degrees) / Ref_Encoder;
              MASE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder;
              MAID = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder;
              MAIE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder;
            }
  
            if(absoluteYaw>=135 && absoluteYaw<=224.999)
            {
              MASE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder;
              MAIE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder;
              MASD = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder;
              MAID = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder;
            }
            if(absoluteYaw>=225 && absoluteYaw<=314.999)
            {
              MAIE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder;
              MAID = (absoluteYaw+JoyRight_degrees) / Ref_Encoder;
              MASE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder;
              MASD = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder;
            }
          }
          if((JoyRight_degrees>=(-45) && JoyRight_degrees<=44.999) && (JoyLeft_degrees>=(-45) && JoyLeft_degrees<=44.999))
          {
            if(absoluteYaw>=315 || absoluteYaw<=44.999)
            {
              MAIE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder;
              MAID = (absoluteYaw+JoyRight_degrees) / Ref_Encoder;
              MASD = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder;
              MASE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder;
            }
        
            if(absoluteYaw>=45 && absoluteYaw<=134.999)
            {
              MASD = (absoluteYaw+JoyRight_degrees) / Ref_Encoder;
              MAID = (absoluteYaw+JoyRight_degrees) / Ref_Encoder;
              MAIE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder;
              MASE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder;
            }
  
            if(absoluteYaw>=135 && absoluteYaw<=224.999)
            {
              MASD = (absoluteYaw+JoyRight_degrees) / Ref_Encoder;
              MASE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder;
              MAID = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder;
              MAIE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder;
            }

            if(absoluteYaw>=225 && absoluteYaw<=314.999)
            {
              MASE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder;
              MAIE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder;
              MASD = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder;
              MAID = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder;
            }
          }
        }

        AMIE.getPIDController().setReference(MAIE, ControlType.kPosition);
        AMID.getPIDController().setReference(MAID, ControlType.kPosition);
        AMSE.getPIDController().setReference(MASE, ControlType.kPosition);
        AMSD.getPIDController().setReference(MASD, ControlType.kPosition);

        MTSD = Velocidade;
        MTID = Velocidade;
        MTIE = Velocidade;
        MTSE = Velocidade;
      }
      else
      {
        if(controller.getAButton()==false)  
        {
          SET_ALL_PID_P(AMIE, AMID, AMSE, AMSD, 0);
        }
        Var_Ctr[1]=false;
      }
    }

    if(controller.getAButton()==true)
    {
      SET_ALL_PID_P(AMIE, AMID, AMSE, AMSD, Ref_PID_P);

      AMIE.getPIDController().setReference(0, ControlType.kPosition);
      AMID.getPIDController().setReference(0, ControlType.kPosition);
      AMSE.getPIDController().setReference(0, ControlType.kPosition);
      AMSD.getPIDController().setReference(0, ControlType.kPosition);
    }

    SmartDashboard.putNumber("PIGEON Yaw", Yaw);
    SmartDashboard.putNumber("PIGEON Pitch", Pitch);
    SmartDashboard.putNumber("PIGEON Roll", Roll);
    SmartDashboard.putNumber("YAW ABSOLUTO", absoluteYaw);
    SmartDashboard.putNumber("Joystick Left", JoyLeft_degrees);
    SmartDashboard.putNumber("Joystick Right", JoyRight_degrees);

    TMSD.set(MTSD);
    TMID.set(MTID);
    TMIE.set(MTIE);
    TMSE.set(MTSE);
  }

  @Override
  public void disabledInit() 
  {}

  @Override
  public void disabledPeriodic() 
  {
    AMIE.setIdleMode(IdleMode.kCoast);
    TMIE.setIdleMode(IdleMode.kCoast);
    AMSE.setIdleMode(IdleMode.kCoast);
    TMSE.setIdleMode(IdleMode.kCoast);
    AMSD.setIdleMode(IdleMode.kCoast);
    TMSD.setIdleMode(IdleMode.kCoast);
    AMID.setIdleMode(IdleMode.kCoast);
    TMID.setIdleMode(IdleMode.kCoast);
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
