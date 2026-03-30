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

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.CANcoder;

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

  private CANcoder CANC_SE;
  private CANcoder CANC_SD;
  private CANcoder CANC_IE;
  private CANcoder CANC_ID;

  private static XboxController controller;

  private Pigeon2 Pigeon;

  private NetworkTable LimeTable;
  private int PipelineSelect = 1;
  private int LEDMODE=1;

  private static final String DashDefault  = "0";
  private static final String DashOpt1     = "1";
  private static final String DashOpt2     = "2";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private static final String Pipe0  = "0";
  private static final String Pipe1  = "1";
  private static final String Pipe2  = "2";
  private String m_PipeSelect;
  private final SendableChooser<String> m_Pipe = new SendableChooser<>();

  private static final String LimeLED0  = "0";
  private static final String LimeLED1  = "1";
  private static final String LimeLED2  = "2";
  private static final String LimeLED3  = "3";
  
  private String m_LimeLEDSelect;
  private final SendableChooser<String> m_LimeLED = new SendableChooser<>();
  double VMotor=0;

  double Direcao=0;
  double Translacao=0;
  int Automode=0;

  private double Ref_Encoder_SD = 16.417020784085121;     // Referência do Encoder: 360°/21.38 = 16.83816651075772 (21.38 = 360° do Encoder)  21,928461
  private double Ref_Encoder_SE = 16.744341807830770;      // 21,4998
  private double Ref_Encoder_IE = 16.762829431505402;     // 21,476088
  private double Ref_Encoder_ID = 16.417020784085121;     //21,928461

  private double Ref_PID_P = 0.25;                     // Referência Proporcional PID: Valor de SetPoint da Proporcional PID
  private double Ref_Min_Max_Wrapping = 0.3;           // Range Máximo e Minimo PID Wrapping: Valor minimo e máximo que vai sair para o motor

  private boolean[] Var_Ctr = new boolean[5];         // Variaveis de controle

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

  public static void SPARKMAX_CONFIG_ANGULO (CANSparkMax Motor, double Ref_Enc, double Min_Max_Wpg)   // Configura o PID, o range de saida e o Modo Wrapping
  {
    Motor.getPIDController().setP(0.2);
    Motor.getPIDController().setI(0);
    Motor.getPIDController().setD(0);
    Motor.getPIDController().setOutputRange(Min_Max_Wpg*(-1), Min_Max_Wpg);
    Motor.getPIDController().setPositionPIDWrappingEnabled(true);
    Motor.getPIDController().setPositionPIDWrappingMaxInput(Ref_Enc);
    Motor.getPIDController().setPositionPIDWrappingMinInput(0);
  }

  public static void SET_ALL_PID_P(CANSparkMax Inf_Esq, CANSparkMax Inf_Dir, CANSparkMax Sup_Esq, CANSparkMax Sup_Dir, double PID_P)  // Configura o P de todos os motores
  {
    Inf_Esq.getPIDController().setP(PID_P);
    Inf_Dir.getPIDController().setP(PID_P);
    Sup_Esq.getPIDController().setP(PID_P);
    Sup_Dir.getPIDController().setP(PID_P);
  }

  public static double ALINHAMENTO(CANSparkMax Motor1, CANSparkMax Motor2, CANSparkMax Motor3, CANSparkMax Motor4, double ABS_Alvo, double Angle_Setpoint)
  {
    double v_Motor=0;

    if(ABS_Alvo>(Angle_Setpoint+20)) v_Motor=-0.2;
    if(ABS_Alvo<(Angle_Setpoint-20)) v_Motor=0.2;
    if(ABS_Alvo>(Angle_Setpoint+1) && ABS_Alvo<=(Angle_Setpoint+20)) v_Motor = -0.035;
    if(ABS_Alvo>=(Angle_Setpoint-21) && ABS_Alvo<=(Angle_Setpoint-1)) v_Motor = 0.035;
    if(ABS_Alvo>(Angle_Setpoint-1) && ABS_Alvo<=(Angle_Setpoint+1))
    {
      v_Motor = 0;

      Motor1.getEncoder().setPosition(0);
      Motor2.getEncoder().setPosition(0);
      Motor3.getEncoder().setPosition(0);
      Motor4.getEncoder().setPosition(0);
    }
    return v_Motor;
  }

  @Override
  public void robotInit() 
  {
    controller = new XboxController(0);
    Pigeon = new Pigeon2(13);
    
    TMIE = new CANSparkMax(3,MotorType.kBrushless);
    AMIE = new CANSparkMax(4,MotorType.kBrushless);
    AMIE.setInverted(true);
    CANC_IE = new CANcoder(9);

    TMID = new CANSparkMax(1,MotorType.kBrushless);
    AMID = new CANSparkMax(2,MotorType.kBrushless);
    AMID.setInverted(true);
    CANC_ID = new CANcoder(12);

    TMSE = new CANSparkMax(5,MotorType.kBrushless);
    AMSE = new CANSparkMax(6,MotorType.kBrushless);
    AMSE.setInverted(true);
    CANC_SE = new CANcoder(10);

    TMSD = new CANSparkMax(7,MotorType.kBrushless);
    AMSD = new CANSparkMax(8,MotorType.kBrushless);
    AMSD.setInverted(true);
    CANC_SD = new CANcoder(11);

    LimeTable = NetworkTableInstance.getDefault().getTable("limelight");

    SPARKMAX_CONFIG_TRACAO(TMIE);
    SPARKMAX_CONFIG_TRACAO(TMID);
    SPARKMAX_CONFIG_TRACAO(TMSD);
    SPARKMAX_CONFIG_TRACAO(TMSE);

    SPARKMAX_CONFIG_ANGULO(AMIE, 21.476088, Ref_Min_Max_Wrapping);
    SPARKMAX_CONFIG_ANGULO(AMID, 21.928461, Ref_Min_Max_Wrapping);
    SPARKMAX_CONFIG_ANGULO(AMSD, 21.928461, Ref_Min_Max_Wrapping);
    SPARKMAX_CONFIG_ANGULO(AMSE, 21.476088, Ref_Min_Max_Wrapping);

    TMIE.getEncoder().setPosition(0);
    AMIE.getEncoder().setPosition(0);

    TMID.getEncoder().setPosition(0);
    AMID.getEncoder().setPosition(0);

    TMSE.getEncoder().setPosition(0);
    AMSE.getEncoder().setPosition(0);

    TMSD.getEncoder().setPosition(0);
    AMSD.getEncoder().setPosition(0);

    // double ABS_CSD = ((CANC_SD.getAbsolutePosition().getValueAsDouble()*360) % 360);
    // double ABS_CID = ((CANC_ID.getAbsolutePosition().getValueAsDouble()*360) % 360);
    // double ABS_CSE = ((CANC_SE.getAbsolutePosition().getValueAsDouble()*360) % 360);
    // double ABS_CIE = ((CANC_IE.getAbsolutePosition().getValueAsDouble()*360) % 360);

    // if (ABS_CSD<0) ABS_CSD += 360;
    // if (ABS_CID<0) ABS_CID += 360;
    // if (ABS_CSE<0) ABS_CSE += 360;
    // if (ABS_CIE<0) ABS_CIE += 360;

    // AMIE.getEncoder().setPosition(ABS_CIE/Ref_Encoder_IE);
    // AMID.getEncoder().setPosition(ABS_CID/Ref_Encoder_ID);
    // AMSE.getEncoder().setPosition(ABS_CSE/Ref_Encoder_SE);
    // AMSD.getEncoder().setPosition(ABS_CSD/Ref_Encoder_SD);

    Pigeon.setYaw(0);

    m_chooser.setDefaultOption("Auto: Inativo", DashDefault);      // Cria o botão principal
    m_chooser.addOption("Auto: Move Front", DashOpt1);                // Adiciona botões a matriz
    m_chooser.addOption("Auto: Move Left", DashOpt2);                // Adiciona botões a matriz
    SmartDashboard.putData("MODO AUTONOMO SELECT:", m_chooser);
    SmartDashboard.putData(CommandScheduler.getInstance());

    m_Pipe.setDefaultOption("PIPE: Nota", Pipe0);
    m_Pipe.addOption("PIPE: Tags", Pipe1);
    m_Pipe.addOption("PIPE: Indefinida", Pipe2);
    SmartDashboard.putData("PIPELINE SELECT:", m_Pipe);
    SmartDashboard.putData(CommandScheduler.getInstance());

    m_LimeLED.setDefaultOption("MODO: Default", LimeLED0);
    m_LimeLED.addOption("MODO: Off", LimeLED1);
    m_LimeLED.addOption("MODO: On", LimeLED2);
    m_LimeLED.addOption("MODO: On Process", LimeLED3);
    SmartDashboard.putData("LIMELED MODE:", m_LimeLED);
    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  @Override
  public void robotPeriodic() 
  {
    double MTSD=0;    // Motor Tração Superior Direito
    double MTID=0;    // Motor Tração Inferior Direito
    double MTSE=0;    // Motor Tração Superior Esquerdo
    double MTIE=0;    // Motor Tração Inferior Esquerdo

    double MAIE=0;   // Motor Tração Inferior Esquerdo
    double MASE=0;   // Motor Tração Superior Esquerdo
    double MAID=0;   // Motor Tração Inferior Direito
    double MASD=0;   // Motor Tração Superior Direito

    double Velocidade = VMotor;
    double JoyLeft_degrees = Direcao;
    double JoyRight_degrees = Translacao;

    double Yaw    = Pigeon.getYaw().getValueAsDouble();
    double Pitch  = Pigeon.getPitch().getValueAsDouble();
    double Roll   = Pigeon.getRoll().getValueAsDouble();

    Yaw*=(-1);
    double absoluteYaw = Yaw % 360;

    double ABS_CSD = ((CANC_SD.getAbsolutePosition().getValueAsDouble()*360) % 360);
    double ABS_CID = ((CANC_ID.getAbsolutePosition().getValueAsDouble()*360) % 360);
    double ABS_CSE = ((CANC_SE.getAbsolutePosition().getValueAsDouble()*360) % 360);
    double ABS_CIE = ((CANC_IE.getAbsolutePosition().getValueAsDouble()*360) % 360);

    if (ABS_CSD<0) ABS_CSD += 360;
    if (ABS_CID<0) ABS_CID += 360;
    if (ABS_CSE<0) ABS_CSE += 360;
    if (ABS_CIE<0) ABS_CIE += 360;

    double TargetX = LimeTable.getEntry("tx").getDouble(0.0);
    double TargetY = LimeTable.getEntry("ty").getDouble(0.0);
    double TargetArea = LimeTable.getEntry("ta").getDouble(0.0);

    m_autoSelected = m_chooser.getSelected();
    switch(m_autoSelected)
    {
      case "0":
      Automode=0;
        break;
      case "1":
      Automode=1;
        break;
      case "2":
      Automode=2;
        break;
      default:
      Automode=(-1);
        break;
    }

    m_PipeSelect = m_Pipe.getSelected();
    switch(m_PipeSelect)
    {
      case "0":
        PipelineSelect=0;
        break;
      case "1":
        PipelineSelect=1;
        break;
      case "2":
        PipelineSelect=2;
        break;
      default:
        PipelineSelect=1;
        break;
    }
    
    m_LimeLEDSelect = m_LimeLED.getSelected();
    switch(m_LimeLEDSelect)
    {
      case "0":
        LEDMODE=0;
        break;
      case "1":
        LEDMODE=1;
        break;
      case "2":
        LEDMODE=2;
        break;
      case "3":
        LEDMODE=3;
        break;
      default:
        LEDMODE=1;
        break;
    }

    if (absoluteYaw<0) absoluteYaw += 360;          // Lógica de retorno ao cruzar os 360° ou decrementar de 0°
  if(controller.getYButton()==false)
  {
    if ((Analog_Stimulated(4, 5, 0.15)==true) && (Analog_Stimulated(0, 1, 0.15)==false) || Var_Ctr[3]==true) 
    {
      if(Var_Ctr[0]==false)
      {
        AMID.getPIDController().setP(Ref_PID_P);
        AMIE.getPIDController().setP(Ref_PID_P);
        AMSD.getPIDController().setP(Ref_PID_P);
        AMSE.getPIDController().setP(Ref_PID_P);

        AMID.getPIDController().setReference(315 / Ref_Encoder_ID, ControlType.kPosition);
        AMIE.getPIDController().setReference(225 / Ref_Encoder_IE, ControlType.kPosition);
        AMSD.getPIDController().setReference(45 / Ref_Encoder_SD, ControlType.kPosition);
        AMSE.getPIDController().setReference(135 / Ref_Encoder_SE, ControlType.kPosition);

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
      if (Analog_Stimulated(0, 1, 0.15) || Var_Ctr[2]==true) 
      {
        if(Var_Ctr[1]==false)
        {
          SET_ALL_PID_P(AMIE, AMID, AMSE, AMSD, Ref_PID_P);
          Var_Ctr[1]=true;
        }

        MAIE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_IE;
        MAID = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_ID;
        MASE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_SE;
        MASD = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_SD;

        if(controller.getRawAxis(4) >= 0.15 || controller.getRawAxis(4) <= -0.15 || Var_Ctr[3]==true)
        {
          if((JoyRight_degrees>=135 || JoyRight_degrees<=-134.999) && (JoyLeft_degrees>=135 || JoyLeft_degrees<=-134.999))
          {
            if(absoluteYaw>=315 || absoluteYaw<=44.999)
            {
              MASE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_SE;
              MASD = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_SD;
              MAID = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_ID;
              MAIE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_IE;
            }
        
            if(absoluteYaw>=45 && absoluteYaw<=134.999)
            {
              MAIE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_IE;
              MASE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_SE;
              MASD = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_SD;
              MAID = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_ID;
            }
  
            if(absoluteYaw>=135 && absoluteYaw<=224.999)
            {
              MAIE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_IE;
              MAID = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_ID;
              MASD = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_SD;
              MASE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_SE;
            }
            if(absoluteYaw>=225 && absoluteYaw<=314.999)
            {
              MASD = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_SD;
              MAID = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_ID;
              MASE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_SE;
              MAIE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_IE;
            }
          }
          if((JoyRight_degrees>=(-135) && JoyRight_degrees<=(-44.999)) && (JoyLeft_degrees>=(-135) && JoyLeft_degrees<=(-44.999)))
          {
            if(absoluteYaw>=315 || absoluteYaw<=44.999)
            {
              MASE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_SE;
              MAIE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_IE;
              MAID = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_ID;
              MASD = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_SD;
            }
        
            if(absoluteYaw>=45 && absoluteYaw<=134.999)
            {
              MAIE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_IE;
              MAID = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_ID;
              MASD = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_SD;
              MASE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_SE;
            }
  
            if(absoluteYaw>=135 && absoluteYaw<=224.999)
            {
              MASD = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_SD;
              MAID = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_ID;
              MASE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_SE;
              MAIE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_IE;
            }
            if(absoluteYaw>=225 && absoluteYaw<=314.999)
            {
              MASD = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_SD;
              MASE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_SE;
              MAIE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_IE;
              MAID = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_ID;
            }
          }
          if((JoyRight_degrees>=45 && JoyRight_degrees<=134.999) && (JoyLeft_degrees>=45 && JoyLeft_degrees<=134.999))
          {
            if(absoluteYaw>=315 || absoluteYaw<=44.999)
            {
              MASD = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_SD;
              MAID = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_ID;
              MAIE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_IE;
              MASE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_SE;
            }
        
            if(absoluteYaw>=45 && absoluteYaw<=134.999)
            {
              MASD = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_SD;
              MASE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_SE;
              MAID = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_ID;
              MAIE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_IE;
            }
  
            if(absoluteYaw>=135 && absoluteYaw<=224.999)
            {
              MASE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_SE;
              MAIE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_IE;
              MASD = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_SD;
              MAID = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_ID;
            }
            if(absoluteYaw>=225 && absoluteYaw<=314.999)
            {
              MAIE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_IE;
              MAID = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_ID;
              MASE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_SE;
              MASD = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_SD;
            }
          }
          if((JoyRight_degrees>=(-45) && JoyRight_degrees<=44.999) && (JoyLeft_degrees>=(-45) && JoyLeft_degrees<=44.999))
          {
            if(absoluteYaw>=315 || absoluteYaw<=44.999)
            {
              MAIE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_IE;
              MAID = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_ID;
              MASD = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_SD;
              MASE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_SE;
            }
        
            if(absoluteYaw>=45 && absoluteYaw<=134.999)
            {
              MASD = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_SD;
              MAID = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_IE;
              MAIE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_IE;
              MASE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_SE;
            }
  
            if(absoluteYaw>=135 && absoluteYaw<=224.999)
            {
              MASD = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_SD;
              MASE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_SE;
              MAID = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_ID;
              MAIE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_IE;
            }

            if(absoluteYaw>=225 && absoluteYaw<=314.999)
            {
              MASE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_SE;
              MAIE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_IE;
              MASD = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_SD;
              MAID = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_ID;
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
  }
    if(controller.getAButton()==true)
    {
      SET_ALL_PID_P(AMIE, AMID, AMSE, AMSD, Ref_PID_P);

      AMIE.getPIDController().setReference(0, ControlType.kPosition);
      AMID.getPIDController().setReference(0, ControlType.kPosition);
      AMSE.getPIDController().setReference(0, ControlType.kPosition);
      AMSD.getPIDController().setReference(0, ControlType.kPosition);
    }

    if(controller.getYButton()==true)
    {
      // AMSD.set(ALINHAMENTO(AMSD, AMSE, AMID, AMIE, ABS_CSD, 168.75));
      // AMSE.set(ALINHAMENTO(AMSD, AMSE, AMID, AMIE, ABS_CSE, 222));
      // AMIE.set(ALINHAMENTO(AMSD, AMSE, AMID, AMIE, ABS_CIE, 158.8));
      // AMID.set(ALINHAMENTO(AMSD, AMSE, AMID, AMIE, ABS_CID, 41.7));
      if(Var_Ctr[4]==false)             //  ALINHAMENTO LIMELIGHT
      {
        AMID.getPIDController().setP(Ref_PID_P);
        AMIE.getPIDController().setP(Ref_PID_P);
        AMSD.getPIDController().setP(Ref_PID_P);
        AMSE.getPIDController().setP(Ref_PID_P);

        AMID.getPIDController().setReference(315 / Ref_Encoder_ID, ControlType.kPosition);
        AMIE.getPIDController().setReference(225 / Ref_Encoder_IE, ControlType.kPosition);
        AMSD.getPIDController().setReference(45 / Ref_Encoder_SD, ControlType.kPosition);
        AMSE.getPIDController().setReference(135 / Ref_Encoder_SE, ControlType.kPosition);

        Var_Ctr[4]=true;
      }

      if(TargetArea!=0 && !(TargetX>-1 && TargetX<1))
      {
        if(TargetX<-13.5) Velocidade=-0.2;
        if(TargetX>13.5) Velocidade=0.2;
        if(TargetX>=-13.5 && TargetX<=-1) Velocidade=-0.05;
        if(TargetX>1 && TargetX<=13.5) Velocidade=0.05;
        if(TargetX>-1 && TargetX<1) Velocidade=0;
      }
      
      MTSD = Velocidade;
      MTID = Velocidade;
      MTIE = Velocidade;
      MTSE = Velocidade;
    }

   if(controller.getBButton()==true)
    {
      SET_ALL_PID_P(AMIE, AMID, AMSE, AMSD, 0);

      AMIE.getEncoder().setPosition(0);
      AMID.getEncoder().setPosition(0);
      AMSE.getEncoder().setPosition(0);
      AMSD.getEncoder().setPosition(0);
    }

    SmartDashboard.putNumber("PIGEON Yaw", Yaw);
    SmartDashboard.putNumber("PIGEON Pitch", Pitch);
    SmartDashboard.putNumber("PIGEON Roll", Roll);
    SmartDashboard.putNumber("YAW ABSOLUTO", absoluteYaw);
    SmartDashboard.putNumber("Joystick Left", JoyLeft_degrees);
    SmartDashboard.putNumber("Joystick Right", JoyRight_degrees);

    SmartDashboard.putNumber("ENCID", 10.4/Ref_Encoder_ID);
    SmartDashboard.putNumber("ENCIE", 160.6/Ref_Encoder_IE);
    SmartDashboard.putNumber("ENCSE", 202.675/Ref_Encoder_SE);
    SmartDashboard.putNumber("ENCSD", 168.75/Ref_Encoder_SD);

    SmartDashboard.putNumber("ENCODER IE", AMIE.getEncoder().getPosition());
    SmartDashboard.putNumber("ENCODER ID", AMID.getEncoder().getPosition());
    SmartDashboard.putNumber("ENCODER SD", AMSD.getEncoder().getPosition());
    SmartDashboard.putNumber("ENCODER SE", AMSE.getEncoder().getPosition());

    SmartDashboard.putNumber("ABS - CANCODER ID", ABS_CID);
    SmartDashboard.putNumber("ABS - CANCODER SD", ABS_CSD);
    SmartDashboard.putNumber("ABS - CANCODER IE", ABS_CIE);
    SmartDashboard.putNumber("ABS - CANCODER SE", ABS_CSE);

    SmartDashboard.putNumber("CANCODER SD", CANC_SD.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("CANCODER ID", CANC_ID.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("CANCODER IE", CANC_IE.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("CANCODER SE", CANC_SE.getAbsolutePosition().getValueAsDouble());

    SmartDashboard.putNumber("TARGET TX", TargetX);
    SmartDashboard.putNumber("TARGET TY", TargetY);
    SmartDashboard.putNumber("TARGET TA", TargetArea);
    
    LimeTable.getEntry("pipeline").setNumber(PipelineSelect);  // Define o valor da pipeline desejada
    LimeTable.getEntry("ledMode").setNumber(LEDMODE);          // Modo do Led definido para 1

    TMSD.set(MTSD);
    TMID.set(MTID);
    TMIE.set(MTIE);
    TMSE.set(MTSE);
  }

  @Override
  public void autonomousInit() 
  {
    Var_Ctr[0]=false;
    Var_Ctr[1]=false;
    Var_Ctr[2]=false;
    Var_Ctr[3]=false;

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
  public void autonomousPeriodic() 
  {
    if(Automode==0)
    {
      VMotor=0;
      Direcao=0;
      Translacao=0;
      Var_Ctr[2]=false;
    }
    if(Automode==1)
    {
      VMotor=0.2;
      Direcao=179.999;
      Translacao=0;
      Var_Ctr[2]=true;
    }
    if(Automode==2)
    {
      VMotor=0.5;
      Direcao=-90;
      Translacao=0;
      Var_Ctr[2]=true;
    }
  }

  @Override
  public void teleopInit() 
  {
    Var_Ctr[0]=false;
    Var_Ctr[1]=false;
    Var_Ctr[2]=false;
    Var_Ctr[3]=false;

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
    VMotor = controller.getRawAxis(3);
    Direcao = Math.toDegrees(Math.atan2(controller.getRawAxis(0), controller.getRawAxis(1)));
    Translacao = Math.toDegrees(Math.atan2(controller.getRawAxis(4), controller.getRawAxis(5)));
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

    VMotor = 0;
    Direcao = 0;
    Translacao = 0;
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
