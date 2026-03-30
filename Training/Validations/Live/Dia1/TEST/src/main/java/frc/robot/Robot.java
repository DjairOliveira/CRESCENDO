// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Servo;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;

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

  private DigitalOutput LED_BRANCO;
  private DigitalInput Fim_Curso;
  private AnalogInput Potenciometro;
  private Servo ServoAzul;
  private CANSparkMax MOTOR1_CIM;
  private CANSparkMax MOTOR2_NEO;
  private XboxController Controle;
  private Encoder Encoder_Externo;
  private NetworkTable Lime;

  private Pigeon2 Pigeon;

  int Cont=0;

  boolean CheckBox=false;

  boolean toogle=false;
  boolean Ctr=false;

  @Override
  public void robotInit() 
  {
    LED_BRANCO = new DigitalOutput(9);
    Fim_Curso = new DigitalInput(4);
    Potenciometro = new AnalogInput(3);
    ServoAzul = new Servo(0);
    MOTOR1_CIM = new CANSparkMax(2, CANSparkMax.MotorType.kBrushed);
    MOTOR2_NEO = new CANSparkMax(3, CANSparkMax.MotorType.kBrushless);

    MOTOR1_CIM.setIdleMode(IdleMode.kBrake);
    MOTOR2_NEO.setIdleMode(IdleMode.kCoast);

    Controle = new XboxController(0);

    Encoder_Externo = new Encoder(0, 1);

    MOTOR2_NEO.getEncoder().setPosition(0);

    SmartDashboard.putBoolean("CHECK", CheckBox);

    Pigeon = new Pigeon2(13);

    Lime = NetworkTableInstance.getDefault().getTable("limelight");

  }

  @Override
  public void robotPeriodic() 
  {
    /*
    LED_BRANCO.set(true);
    Timer.delay(2);  // Aguarda 0.5 segundos  
    LED_BRANCO.set(false);
    Timer.delay(2);  // Aguarda 0.5 segundos
    */
    // boolean Leitura=false;

    // Leitura=Fim_Curso.get();

    // if(Leitura == true)
    // {
    //   LED_BRANCO.set(true);
    // }
    // if(Leitura == false)
    // {
    //   LED_BRANCO.set(false);
    // }

      int Enc_Ext = Encoder_Externo.get();
      double Enc_Internal = MOTOR2_NEO.getEncoder().getPosition();
      double[] YPR = new double[3];

      Pigeon.getYawPitchRoll(YPR);
      // Cont++;

      // if(Cont>100)
      // {
      //   System.out.println("Enc Externo: "+Enc_Ext);
      //   System.out.println("Enc Interno NEO: "+Enc_Internal);
      //   Cont=0;
      // }
      NetworkTableInstance inst = NetworkTableInstance.getDefault();
      NetworkTable smartDashboard = inst.getTable("SmartDashboard");

      Boolean CheckBox = smartDashboard.getEntry("CHECK").getBoolean(false);

      SmartDashboard.putNumber("ENCODER INTERNO", Enc_Internal);
      SmartDashboard.putNumber("YAW", YPR[0]);
      SmartDashboard.putNumber("PITCH", YPR[1]);
      SmartDashboard.putNumber("ROLL", YPR[2]);

      // SmartDashboard.putBoolean("CHECK 1", CheckBox);

      if(CheckBox==true)
      {
        LED_BRANCO.set(true);
      }
      else
      {
        LED_BRANCO.set(false);
      }

      Lime.getEntry("pipeline").setNumber(0);
      Lime.getEntry("ledMode").setNumber(1);
   
      /* Opções de configura do LED
     * 0 = Modo padrão definido pelo pipeline
     * 1 = Desliga os LEDs
     * 2 = Acende os LEDs em modo de visão
     * 3 = Acende os LEDs em modo de processamento
     */

      NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
      NetworkTableEntry tx = table.getEntry("tx");
      NetworkTableEntry ty = table.getEntry("ty");
      NetworkTableEntry ta = table.getEntry("ta");
      
      //read values periodically
      double x = tx.getDouble(0.0);
      double y = ty.getDouble(0.0);
      double area = ta.getDouble(0.0);
      
      //post to smart dashboard periodically
      SmartDashboard.putNumber("LimelightX", x);
      SmartDashboard.putNumber("LimelightY", y);
      SmartDashboard.putNumber("LimelightArea", area);

  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() 
  {

  }

  @Override
  public void teleopInit() 
  {

  }

  @Override
  public void teleopPeriodic() 
  {
    int Pot=0;

    Pot=Potenciometro.getValue();
    //System.out.println("Leitura do potenciometro: "+Pot);

    ServoAzul.setAngle(Pot/22.755555555555555555555555555556);
    // MOTOR1_CIM.set(Pot*0.000244140625);
    // MOTOR2_NEO.set(Pot*0.000244140625);
    MOTOR1_CIM.set(Controle.getRawAxis(1));
    MOTOR2_NEO.set(Controle.getRawAxis(5));

    if((Controle.getAButton()==true) && (Ctr==false))
    {
      toogle=!toogle;
      Ctr=true;
    }
    if((Controle.getAButton()==false) && (Ctr==true))
    {
      Ctr=false;
    }

    SmartDashboard.putBoolean("TOOGLE", toogle);
  }

  @Override
  public void disabledInit() {}

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
