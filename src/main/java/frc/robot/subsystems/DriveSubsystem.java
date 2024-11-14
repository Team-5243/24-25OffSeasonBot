// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Utilities;

public class DriveSubsystem extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */
    public CANSparkMax flMotor;
    public CANSparkMax frMotor;
    public CANSparkMax blMotor;
    public CANSparkMax brMotor;
    public RelativeEncoder flEncoder;
    public RelativeEncoder frEncoder;
    public DifferentialDrive diffDrive;
    public Voltage volts;
    public SysIdRoutine sysid;
    public RamseteController rController;
    public AHRS gyro;


    public DriveSubsystem() {
        flMotor = new CANSparkMax(Constants.FL, MotorType.kBrushless);
        frMotor = new CANSparkMax(Constants.FR, MotorType.kBrushless);
        blMotor = new CANSparkMax(Constants.BL, MotorType.kBrushless);
        brMotor = new CANSparkMax(Constants.BR, MotorType.kBrushless);

        blMotor.follow(flMotor);
        brMotor.follow(frMotor);
        frEncoder = frMotor.getEncoder();
        flEncoder = flMotor.getEncoder();

        

        diffDrive = new DifferentialDrive(flMotor, frMotor);

        sysid = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(
            voltage -> {
                flMotor.setVoltage(voltage.magnitude());
                frMotor.setVoltage(voltage.magnitude());
            },
            log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-left")
                    .voltage(
                        Units.Volts.of(
                            flMotor.get() * RobotController.getBatteryVoltage()))
                    .linearPosition(Units.Meters.of(Utilities.rotationsToInches(flEncoder.getPosition())))
                    .linearVelocity(
                        Units.MetersPerSecond.of(flEncoder.getVelocity()));
                // Record a frame for the right motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-right")
                    .voltage(
                        Units.Volts.of(frMotor.get() * RobotController.getBatteryVoltage()))
                    .linearPosition(Units.Meters.of(Utilities.rotationsToInches(frEncoder.getPosition())))
                    .linearVelocity(
                        Units.MetersPerSecond.of(frEncoder.getVelocity()));
              }, this));

        rController = new RamseteController();

        gyro = new AHRS();
    }

    /**
     * Example command factory method.
     *
     * @return a command
     */
    public Command exampleMethodCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    /* one-time action goes here */
                });
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a
     * digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void arcade(double xSpeed, double forward) {
        diffDrive.arcadeDrive(xSpeed, forward);
    }

    public void tank(double leftSpeed, double rightSpeed) {
        diffDrive.tankDrive(leftSpeed, rightSpeed);
    }
}
