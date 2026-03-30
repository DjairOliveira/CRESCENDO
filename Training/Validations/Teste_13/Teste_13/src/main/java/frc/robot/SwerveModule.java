package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.ModuleConstants;

public class SwerveModule
{

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private CANEncoder driveEncoder;
    private CANEncoder turningEncoder;

    private final PIDController turningPidController;

    private final CANCoder AbsoluteEncoder;

    private final boolean AbsoluteEncoderReversed;
    private final double AbsoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed, int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed)
    {
        this.AbsoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.AbsoluteEncoderReversed = absoluteEncoderReversed;
        AbsoluteEncoder = new CANCoder(absoluteEncoderId);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kturningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        
        resetEncoders();

    }

public double getDrivePosition()
{
    return driveEncoder.getPosition();
}
public double getTurningPosition()
{
    return turningEncoder.getPosition();
}

public double getDriveVelocity()
{
    return driveEncoder.getVelocity();
}
public double getTurningVelocity()
{
    return turningEncoder.getVelocity();
}

public double getAbsoluteEncoderRad()
{
    double angle = AbsoluteEncoder.getAbsolutePosition();
    angle *= (Math.PI/180);
    angle -= AbsoluteEncoderOffsetRad;
    return angle * (AbsoluteEncoderReversed ? -1.0:1.0);
}
public void resetEncoders()
{
    driveEncoder.setPosition(0);
    turningEncoder.setPosition(getAbsoluteEncoderRad());
}

public SwerveModuleState getState()
{
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
}

public void setDesiredState(SwerveModuleState state)
{
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
        stop();
        return;
    }
    state = SwerveModuleState.optimize(state, getState().angle);
    driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    SmartDashboard.putString("Swerve[" + AbsoluteEncoder.getAbsolutePosition() + "] state", state.toString());
}

public void stop() {
    driveMotor.set(0);
    turningMotor.set(0);
}
}

