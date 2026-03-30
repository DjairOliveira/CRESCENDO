// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

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

   CANSparkMax M1;
   CANSparkMax M2;
   CANSparkMax M3;
   CANSparkMax M4;
  
   XboxController controle;
  @Override
  public void robotInit() {

    M1 = new CANSparkMax(1, MotorType.kBrushed);
    M2 = new CANSparkMax(2, MotorType.kBrushed);
    M3 = new CANSparkMax(3, MotorType.kBrushed);
    M4 = new CANSparkMax(4, MotorType.kBrushed);

    controle = new XboxController(0);
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
  public void teleopPeriodic() {
    double Valor_M=0;
    double Valor_Esq=0;
    double Valor_Dir=0;

    Valor_M = controle.getLeftTriggerAxis()*(-1) + controle.getRightTriggerAxis();
    if(Valor_M>1) Valor_M=1;
    if(Valor_M<(-1)) Valor_M=(-1);
    
    if(controle.getLeftX()>0.2) Valor_Dir=controle.getLeftX()/2.5;
    if(controle.getLeftX()<(-0.2)) Valor_Esq=controle.getLeftX()/2.5;

    if(controle.getLeftTriggerAxis()>0.2 || controle.getLeftTriggerAxis()<(-0.2))
    {
      M1.set(Valor_M-Valor_Dir);
      M2.set(Valor_M-Valor_Dir);
      M3.set((Valor_M*(-1)) - Valor_Esq);
      M4.set((Valor_M*(-1))  - Valor_Esq);
    }
    if(controle.getLeftTriggerAxis()<=0.2 && controle.getLeftTriggerAxis()>=(-0.2))
    {
      M1.set(Valor_M-Valor_Dir+(Valor_Esq*(-1)));
      M2.set(Valor_M-Valor_Dir+(Valor_Esq*(-1)));
      M3.set((Valor_M*(-1)) - Valor_Esq+(Valor_Dir*(-1)));
      M4.set((Valor_M*(-1))  - Valor_Esq+(Valor_Dir*(-1)));
    }



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
