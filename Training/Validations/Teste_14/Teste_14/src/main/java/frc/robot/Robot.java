// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.TimedRobot;

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
   private XboxController driverController;
   private SwerveDriveKinematics kinematics;
   private SwerveDriveOdometry odometry;
   private SwerveDrive swerveDrive;

   private SwerveModule frontLeftModule;
   private SwerveModule frontRightModule;
   private SwerveModule rearLeftModule;
   private SwerveModule rearRightModule;

  @Override
  public void robotInit() 
  {
    driverController = new XboxController(0);

    // Configurar a cinemática do chassi swerve
    // Substitua as dimensões pelos valores específicos do seu chassi
    kinematics = new SwerveDriveKinematics(
        new Translation2d(0.5, 0.5),  // Posição da roda traseira esquerda
        new Translation2d(0.5, -0.5), // Posição da roda traseira direita
        new Translation2d(-0.5, 0.5), // Posição da roda dianteira esquerda
        new Translation2d(-0.5, -0.5)  // Posição da roda dianteira direita
    );

    // Configurar a odometria do chassi swerve
    // Substitua as posições e a rotação inicial pelos valores específicos do seu robô
    odometry = new SwerveDriveOdometry(kinematics, getRotation2d(), null, new Pose2d(0, 0, Rotation2d.fromDegrees(0)));



        // Configurar os módulos swerve
        frontLeftModule = new SwerveModule(1, 2); // Substitua pelos canais dos seus motores e encoders
        frontRightModule = new SwerveModule(3, 4);
        rearLeftModule = new SwerveModule(5, 6);
        rearRightModule = new SwerveModule(7, 8);

    swerveDrive = new SwerveDrive(kinematics, 
    frontLeftModule::setDesiredState,
    frontRightModule::setDesiredState,
    rearLeftModule::setDesiredState,
    rearRightModule::setDesiredState
);

    /*
    // Configurar o chassi swerve
    swerveDrive = new SwerveDrive(kinematics, wheelSpeeds -> {
    // Substitua isso pela lógica real de controle dos motores swerve
    // wheelSpeeds contém as velocidades angulares para cada roda
    // Exemplo: frontLeftMotor.set(wheelSpeeds.frontLeftRadiansPerSecond);
    });
*/
    // Configurar o período de atualização da odometria
    //setPeriod(0.02); // Defina isso para corresponder ao seu período de controle (50 Hz neste exemplo)
    

    
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
    double forward = -driverController.getLeftY(); // Inverter, se necessário
    double strafe = driverController.getLeftX();
    double rotation = driverController.getRawAxis(4); // Eixo dos gatilhos

    // Controle o chassi swerve
    swerveDrive.driveCartesian(forward, strafe, rotation);

    // Atualizar a odometria
    odometry.update(getRotation2d(), swerveDrive.getSwerveModuleStates());
    
    // Enviar dados de odometria para o SmartDashboard (opcional)
    SmartDashboard.putNumber("X", odometry.getPoseMeters().getTranslation().getX());
    SmartDashboard.putNumber("Y", odometry.getPoseMeters().getTranslation().getY());
    SmartDashboard.putNumber("Heading", odometry.getPoseMeters().getRotation().getDegrees());

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

  private Rotation2d getRotation2d() {
    // Substitua isso pela leitura real do giroscópio ou outro sensor de orientação
    return Rotation2d.fromDegrees(0);
}
}
