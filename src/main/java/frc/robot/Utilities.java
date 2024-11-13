package frc.robot;

public class Utilities {
    public static double inchesToRotations(double inches) {
        return (inches / (Math.PI * Constants.WHEEL_DIAMETER)) * Constants.GEARBOX_RATIO;
    }
}
