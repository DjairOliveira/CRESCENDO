// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogInput;
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

  private DigitalInput InD_Pin0;          // Instance Entrada Digital 0
  private DigitalOutput OutD_Pin1;        // Instance Saida Digital 1

  private AnalogInput InA_Pin0;           // Instance Entrada Analógica 0

  @Override
  public void robotInit()
  {
    InD_Pin0 = new  DigitalInput(0);  // Define o Pino 0 como Entrada Digital
    OutD_Pin1 = new DigitalOutput(1); // Define o Pino 1 como saida Digital
    InA_Pin0 = new AnalogInput(0);    // Define o Pino 0 como Entrada Analógica
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
  public void teleopInit() {}

  @Override
  public void teleopPeriodic()
  {
    boolean ReadDig0 = InD_Pin0.get();  // Obtem o valor da entrada Digital
    int ReadAn0 = InA_Pin0.getValue();  // Obtem o valor da entrada Analógica
    
    OutD_Pin1.set(true);          // Liga o Pino 1
    Timer.delay(0.5);           // Tempo de 0.5s
    OutD_Pin1.set(false);         // Desliga o Pino 1
    Timer.delay(0.5);           // Tempo de 0.5s

    System.out.println("Leitura Digital = "+ReadDig0);
    System.out.println("Leitura Analógica = "+ReadAn0);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic()
  {
    OutD_Pin1.set(false);         // Desliga o Pino 1
    //System.out.println("Robô em modo de espera");
    //Timer.delay(2);
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
