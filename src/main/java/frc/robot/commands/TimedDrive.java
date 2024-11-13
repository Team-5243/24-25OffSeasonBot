// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class TimedDrive extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final DriveSubsystem m_subsystem;
    public double yaw;
    public double turn = 0;
    public double time;
    public final double tolerance = .75;
    public Timer timer = new Timer();

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public TimedDrive(DriveSubsystem subsystem, double time) {
        m_subsystem = subsystem;
        m_subsystem.gyro.reset();
        this.time = time;
        timer.restart();
        
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.restart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        yaw = m_subsystem.gyro.getYaw();
        if (yaw + tolerance < 0)
            turn += 0.0005;
        if (yaw - tolerance > 0)
            turn -= 0.0005;
        
        m_subsystem.arcade(turn, -0.5);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.get() > time;
    }
}
