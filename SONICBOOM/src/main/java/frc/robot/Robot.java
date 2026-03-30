// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ConstChassy;
import frc.robot.Constants.ConstController;
// import frc.robot.Constants.ConstShutter;

import java.io.File;
import java.io.IOException;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.OdometryThread;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import swervelib.parser.SwerveParser;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot
{
  XboxController m_controle = new XboxController(1);
  private static Robot   instance;
  private        Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Timer disabledTimer;

  private boolean controlVar=false;
  private int contagem=0;
  private boolean ctrAjuste1;
  private boolean ctrAjuste2;
  private int cont=0;

  Pose2d poseA = new Pose2d();
  Pose2d poseB = new Pose2d();

  //   private static final Translation2d[] MODULE_POSITIONS = {
  //     new Translation2d(0.9, 0.9),  // Módulo frontal direito
  //     new Translation2d(0.9, -0.9), // Módulo traseiro direito
  //     new Translation2d(-0.9, 0.9), // Módulo frontal esquerdo
  //     new Translation2d(-0.9, -0.9) // Módulo traseiro esquerdo
  //   };
  //   private SwerveDriveKinematics nKinematics = new SwerveDriveKinematics(MODULE_POSITIONS);

  // Rotation2d nRotation2d = new Rotation2d();

  // private static final double angleDegrees = 90.0; // ângulo do módulo em graus
  // private static final double distanceMeters = 1.0; // distância percorrida em metros

  // Instância de SwerveModulePosition
  //private SwerveModulePosition[] nModulePosition = new SwerveModulePosition[];

  //Pose2d initialPose = new Pose2d(0.0, 0.0, new Rotation2d(0.0)); // Pose inicial (0,0) com ângulo 0

  StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose2d.struct).publish();
  StructArrayPublisher<Pose2d> arrayPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("MyPoseArray", Pose2d.struct).publish();

  public Robot()
  {
    instance = this;
  }

  public static Robot getInstance()
  {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit()
  {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();

    ConstChassy.btn_modeUp.set(false);
    ConstChassy.btn_modeDw.set(false);
    ConstChassy.btn_veloUp.set(false);
    ConstChassy.btn_veloUp.set(false);
    ConstChassy.btn_onOff.set(false);
    ConstChassy.btn_pause.set(false);

    ConstChassy.pigeon.setYaw(0);
  
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    if(RobotContainer.engatilhado==true){
      ConstController.controllerCoDriver.setRumble(RumbleType.kRightRumble, 0.5);
    }

    publisher.set(poseA);
    arrayPublisher.set(new Pose2d[] {poseA, poseB});
    
    // Atualize os valores das poses
    //poseA = odometry.getPoseMeters();  // Supondo que você está usando WPILib odometry

    // m_limeLight.periodic();

    // Alliance alliance = DriverStation.getAlliance().get();

    // if (alliance == Alliance.Red){
    //     m_limeLight.setPipeline(5);
    // } else if (alliance == Alliance.Blue){
    //     m_limeLight.setPipeline(6);
    // }

    /*
     * 
     * O exemplo abaixo, ilustra por exemplo um sensor, visto que ao tentar simular com um controle
     * no modo autonomo, o controle fica inutilizado.
     * A lógica abaixo utiliza um contador como sensor, de forma que ao passar o tempo setado, informa durante o autonomo
     * qual PATH o Robo deve executar PATH1 ou PATH2.
     */
    cont++;
    //RobotContainer.Cond = m_controle.getBackButton(); Não deu certo
    if(cont>50){
      RobotContainer.Cond = true;
    }
    else{
      RobotContainer.Cond = false;
    }
    
    SmartDashboard.putBoolean("Sensor ITK", ConstChassy.sensor_Itk.get());
    // SmartDashboard.putNumber("Sensor Turret", ConstChassy.turret_Hor.getEncoder().getPosition());
    // SmartDashboard.putNumber("Sensor Shutter Right", ConstShutter.shutter_inc_right.getEncoder().getPosition());
    // SmartDashboard.putNumber("Sensor Shutter Left", ConstShutter.shutter_inc_left.getEncoder().getPosition());
    SmartDashboard.putNumber("Contador", RobotContainer.timerInterrupt);
    SmartDashboard.putNumber("Pigeon Yaw", ConstChassy.pigeon.getYaw().getValue());
    SmartDashboard.putNumber("Contagem", cont);
    SmartDashboard.putBoolean("Cond", RobotContainer.Cond);
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic()
  {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
    {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
    }
    cont=0;
    // ConstChassy.turret_Hor.setIdleMode(IdleMode.kBrake);
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    }
    cont=0;
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
    
  }

  @Override
  public void teleopInit()
  {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    }
    //m_robotContainer.setDriveMode();
    m_robotContainer.setMotorBrake(true);
    contagem=40;
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic()
  {
    if((joystickStimulated(m_robotContainer.driverXbox.getLeftX()) || joystickStimulated(m_robotContainer.driverXbox.getLeftY()) || joystickStimulated(m_robotContainer.driverXbox.getRightX()) || joystickStimulated(m_robotContainer.driverXbox.getRightY())) && contagem>=40){
      m_robotContainer.setDriveMode("angularVelocity");
      controlVar=false;
      contagem=0;
    }
    
    // if((m_robotContainer.driverXbox.a().getAsBoolean() ||  m_robotContainer.driverXbox.b().getAsBoolean() || m_robotContainer.driverXbox.x().getAsBoolean() || m_robotContainer.driverXbox.y().getAsBoolean()) && controlVar==false){
    //   m_robotContainer.setDriveMode("absolute");
    //   controlVar=true;
    // }
    if(controlVar==true){
      contagem++;
    }
    // if(m_controle.getRightTriggerAxis()>0.5 && ctrAjuste1==false){
    //   ConstShutter.shutter_disp_sup.set(0.05);
    //   ConstShutter.shutter_disp_inf.set(0.3);////+
    //   ConstShutter.shutter_itk.set(-0.5);
    //   ctrAjuste1=true;
    // }
    // if(m_controle.getRightTriggerAxis()<0.5 && ctrAjuste1==true){
    //   ConstShutter.shutter_disp_sup.set(0);
    //   ConstShutter.shutter_disp_inf.set(0);
    //   ConstShutter.shutter_itk.set(0);
    //   ctrAjuste1=false;
    // }

    // if(m_controle.getLeftTriggerAxis()>0.5  && ctrAjuste2==false){
    //   ConstShutter.shutter_disp_sup.set(-0.5);
    //   ConstShutter.shutter_disp_inf.set(-0.5);
    //   ConstShutter.shutter_itk.set(0.2);
    //   ctrAjuste2=true;
    // }
    // if(m_controle.getLeftTriggerAxis()<0.5 && ctrAjuste2==true){
    //   ConstShutter.shutter_disp_sup.set(0);
    //   ConstShutter.shutter_disp_inf.set(0);
    //   ConstShutter.shutter_itk.set(0);
    //   ctrAjuste2=false;
    // }
  }

  @Override
  public void testInit()
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    try
    {
      new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"));
    } catch (IOException e)
    {
      throw new RuntimeException(e);
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic()
  {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit()
  {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic()
  {
  }

  public boolean joystickStimulated(double joystick){
    boolean value=false;
    if(joystick<(-0.08) || joystick>(0.08))
    {
      value=true;
    }else{
      value=false;
    }
    return value;
  }
}
