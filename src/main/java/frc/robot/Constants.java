// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static final int FR = 11;
    public static final int FL = 13;
    public static final int BR = 12;
    public static final int BL = 14;

    public static final Joystick mainStick = new Joystick(0);
    public static final Joystick secondaryStick = new Joystick(1);
    public static XboxController controller = new XboxController(0);

    public static final double WHEEL_DIAMETER = 5.5;
    public static final double GEARBOX_RATIO = 5.95;
}
