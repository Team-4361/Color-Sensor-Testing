package frc.robot;

import static frc.robot.Constants.ACCEPTOR_MOTOR_PORT;
import static frc.robot.Constants.ACCEPTOR_PHOTO_ELECTRIC_PORT;
import static frc.robot.Constants.COLOR_SENSOR_PORT;
import static frc.robot.Constants.DEFAULT_MOTOR_TYPE;
import static frc.robot.Constants.SHOOTER_MOTOR_PORT;
import static frc.robot.Constants.STORAGE_MOTOR_PORT;
import static frc.robot.Constants.STORAGE_PHOTO_ELECTRIC_PORT;

import java.util.concurrent.TimeUnit;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class Robot extends TimedRobot {

    private CANSparkMax acceptorMotor, storageMotor, shooterMotor;
    private Joystick joystick = new Joystick(0);
    private ColorSensorV3 colorSensor;
    private Color color;

    private DigitalInput acceptorSensor, storageSensor;
    private RelativeEncoder acceptorMotorEncoder, storageMotorEncoder;

    private double proximity, acceptorCurrent, storageCurrent, storageRPM, acceptorRPM;

    private int ballsLoaded = 0;

    // Disable if the experimental current system fails, will always say the motor is unstalled.
    private final boolean currentMeasuringEnabled = true;

    // Maximum speed the motors will spin, good for testing.
    private final double motorSpeed = 0.3;

    // Adjust the stall current (Amps) and stall RPM (velocity) based on the motor possibly
    private final double stallCurrent = 80, stallRPM = 2000;

    private final boolean acceptorFlipped = false;
    private final boolean storageFlipped = false;
    private final boolean shooterFlipped = false;

    // This makes it significantly easier to flip the values if they need to be.
    private double getMotorValue(double speed, boolean flipped) {
        if (flipped) {
            return -speed;
        } else {
            return speed;
        }
    }

    @Override public void simulationInit() {}
    @Override public void disabledInit() {}
    @Override public void autonomousInit() {}
    @Override public void testInit() {}
    @Override public void robotPeriodic() {}
    @Override public void simulationPeriodic() {}
    @Override public void disabledPeriodic() {}
    @Override public void autonomousPeriodic() {}

    @Override
    public void robotInit() {
        acceptorMotor = new CANSparkMax(ACCEPTOR_MOTOR_PORT, DEFAULT_MOTOR_TYPE);
        storageMotor = new CANSparkMax(STORAGE_MOTOR_PORT, DEFAULT_MOTOR_TYPE);
        shooterMotor = new CANSparkMax(SHOOTER_MOTOR_PORT, DEFAULT_MOTOR_TYPE);
        colorSensor = new ColorSensorV3(COLOR_SENSOR_PORT);
        acceptorSensor = new DigitalInput(ACCEPTOR_PHOTO_ELECTRIC_PORT);
        storageSensor = new DigitalInput(STORAGE_PHOTO_ELECTRIC_PORT);
        acceptorMotorEncoder = acceptorMotor.getEncoder();
        storageMotorEncoder = storageMotor.getEncoder();

        updateSensors();
    }

    public boolean isStalled(CANSparkMax motor) {
        // If current measuring is enabled and the specified motor RPM is below stall, and current is higher than maximum then its jammed.
        if (currentMeasuringEnabled) {
            if (Math.abs(motor.getEncoder().getVelocity()) < stallRPM && Math.abs(motor.getOutputCurrent()) > stallCurrent) {
                SmartDashboard.putBoolean("jammed", true);
                return true;
            } else {
                SmartDashboard.putBoolean("jammed", false);
                return false;
            }
        } else { 
            // If current measuring disabled, always return false.
            return false;
         }
    }

    public void runMotorTimed(CANSparkMax motor, double speed, int delayTime) {
        new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    long stopTime = System.currentTimeMillis() + delayTime;
                    motor.set(speed);

                    // Add a delay before checking for stall current to reduce the possibility of the
                    // extra power and low speed of starting the motor to activate the protection.
                    TimeUnit.MILLISECONDS.sleep(100);

                    while (System.currentTimeMillis() <= stopTime) {
                        if (!isStalled(motor)) {
                            motor.set(speed);
                        } else {
                            motor.set(0);
                            SmartDashboard.putBoolean("jammed", true);
                        }
                    }
                    motor.set(0);
                } catch (InterruptedException e) {}
            }
        }).start();
    }

    public void activateShooter() {
        runMotorTimed(shooterMotor, getMotorValue(motorSpeed, shooterFlipped), 2000);
        // Possibly run the storage motor slightly as well?
    }

    public void checkBall() {
        new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    if (proximity > 120) {
                        if (color.blue > 0.35) {
                            if (ballsLoaded == 0) {
                                acceptorMotor.set(getMotorValue(motorSpeed, acceptorFlipped));
                                storageMotor.set(getMotorValue(motorSpeed, storageFlipped));

                                // Add a delay before checking for stall current to reduce the possibility of the
                                // extra power and low speed of starting the motor to activate the protection.
                                TimeUnit.MILLISECONDS.sleep(100);

                                while (!storageSensor.get() && !isStalled(acceptorMotor) && !isStalled(storageMotor)) {
                                    acceptorMotor.set(getMotorValue(motorSpeed, acceptorFlipped));
                                    storageMotor.set(getMotorValue(motorSpeed, storageFlipped));
                                }
                            } else if (ballsLoaded == 1) {
                                acceptorMotor.set(getMotorValue(motorSpeed, acceptorFlipped));

                                TimeUnit.MILLISECONDS.sleep(100);
                                
                                while (!acceptorSensor.get() && !isStalled(acceptorMotor)) {
                                    acceptorMotor.set(getMotorValue(motorSpeed, acceptorFlipped));
                                }
                            } else if (ballsLoaded > 1) {
                                // There is more than one ball added to the system, do nothing.
                            }

                            // This should run after the sensor loops are finished, stop the motors.
                            acceptorMotor.set(0);
                            storageMotor.set(0);
                        } else if (color.red > 0.35) {
                            acceptorMotor.set(getMotorValue(-motorSpeed, acceptorFlipped));
                        } else {
                            acceptorMotor.set(0);
                        }
                    } else {
                        acceptorMotor.set(0);
                    }
                } catch (InterruptedException e) {}
            }
        }).start();
    }

    private void updateSensors() {
        color = colorSensor.getColor();
        proximity = colorSensor.getProximity();

        acceptorCurrent = acceptorMotor.getOutputCurrent();
        storageCurrent = storageMotor.getOutputCurrent();

        acceptorRPM = acceptorMotorEncoder.getVelocity();
        storageRPM = storageMotorEncoder.getVelocity();

        // Values that we may need for testing
        SmartDashboard.putNumber("red", color.red);
        SmartDashboard.putNumber("green", color.green);
        SmartDashboard.putNumber("blue", color.blue);
        SmartDashboard.putNumber("proximity", proximity);

        SmartDashboard.putBoolean("acceptor loaded", !acceptorSensor.get());
        SmartDashboard.putBoolean("storage loaded", !storageSensor.get());

        SmartDashboard.putNumber("acceptor amps", acceptorCurrent);
        SmartDashboard.putNumber("storage amps", storageCurrent);

        SmartDashboard.putNumber("acceptor rpm", acceptorRPM);
        SmartDashboard.putNumber("storage rpm", storageRPM);

        SmartDashboard.putNumber("balls loaded", ballsLoaded);

        if (storageSensor.get()) {
            // If the storage sensor (rear) is covered, we know there is a ball inside of it.
            ballsLoaded = 1;

            if (acceptorSensor.get()) {
                // In addition, if the other sensor (front) is covered, there are two balls and can't accept more.
                ballsLoaded = 2;
            }
        } else {
            ballsLoaded = 0;
        }
    }

    @Override
    public void teleopInit() {
        SmartDashboard.putBoolean("storage ready", true);
        SmartDashboard.putBoolean("jammed", false);
    }

    @Override
    public void teleopPeriodic() {
        updateSensors();
        checkBall();

        if (joystick.getRawButtonPressed(9)) {
            activateShooter();
        }
    }
}