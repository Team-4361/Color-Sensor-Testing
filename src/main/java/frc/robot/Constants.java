package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.I2C;

public class Constants {
    // The first motor in the Storage device, used to accept the ball after the sensor is activated.
    public static final int ACCEPTOR_MOTOR_PORT = 7;

    // The second middle motor in the Storage device, used to move the ball inside.
    public static final int STORAGE_MOTOR_PORT = 6;

    // The shooter motor in the Shooter device, runs at Full Speed and shoots the ball.
    public static final int SHOOTER_MOTOR_PORT = 8;

    ////////////////////////////////////////////////////////////////////////////////////////

    // Used to detect the presence of a ball inside the front of the device, mainly used for 2nd entering ball.
    public static final int ACCEPTOR_PHOTO_ELECTRIC_PORT = 1;

    // Used to detect the presence of a ball inside the middle of the device, where the first ball should go.
    public static final int STORAGE_PHOTO_ELECTRIC_PORT = 0;

    ////////////////////////////////////////////////////////////////////////////////////////

    // The port to use for the Color Sensor detection.
    public static final I2C.Port COLOR_SENSOR_PORT = I2C.Port.kOnboard;

    // Adjust based on sensitivity.
    public static final double BLUE_THRESHOLD = 0.30;
    public static final double RED_THRESHOLD = 0.30;
    public static final double PROXIMITY_THRESHOLD = 120;

    /////////////////////////////////////////////////////////////////////////////////////////
    public static final CANSparkMaxLowLevel.MotorType DEFAULT_MOTOR_TYPE = CANSparkMaxLowLevel.MotorType.kBrushless;

    public static final int SHOOT_JOY_PORT = 3;
    public static final int RESET_JOY_PORT = 5;


}
