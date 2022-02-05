package frc.robot;

import static frc.robot.Constants.ACCEPTOR_MOTOR_PORT;
import static frc.robot.Constants.ACCEPTOR_PHOTO_ELECTRIC_PORT;
import static frc.robot.Constants.COLOR_SENSOR_PORT;
import static frc.robot.Constants.DEFAULT_MOTOR_TYPE;
import static frc.robot.Constants.RESET_JOY_PORT;
import static frc.robot.Constants.SHOOTER_MOTOR_PORT;
import static frc.robot.Constants.SHOOT_JOY_PORT;
import static frc.robot.Constants.STORAGE_MOTOR_PORT;
import static frc.robot.Constants.STORAGE_PHOTO_ELECTRIC_PORT;
import static frc.robot.Constants.RED_THRESHOLD;
import static frc.robot.Constants.BLUE_THRESHOLD;
import static frc.robot.Constants.PROXIMITY_THRESHOLD;

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
    private RelativeEncoder acceptorMotorEncoder, storageMotorEncoder, shooterEncoder;

    private double proximity, acceptorCurrent, storageCurrent, storageRPM, acceptorRPM, shooterCurrent, shooterRPM;

    private int ballsLoaded = 0;

    // Disable if the experimental current system fails, will always say the motor is unstalled.
    private final boolean currentMeasuringEnabled = true;

    // Maximum speed the motors will spin, good for testing.
    private final double shootSpeed = 1.0;
    private final double acceptSpeed = 0.3;

    // Adjust the stall current (Amps) and stall RPM (velocity) based on the motor possibly
    private final double stallCurrent = 80, stallRPM = 2000;

    private final boolean acceptorFlipped = true;
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
        shooterEncoder = shooterMotor.getEncoder();

        updateSensors();

        // reset everything
        ballsLoaded = 0;
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
                Thread.currentThread().interrupt();
            }
        }).start();
    }

    boolean shootingDone = false;

    public void activateShooter(double speed) {
        new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    // Change this line before testing or it will not be good!!!!!!
                    runMotorTimed(shooterMotor, getMotorValue(speed, shooterFlipped), 3000);
                    
                    while (Math.abs(shooterEncoder.getVelocity()) < 4000) {
                        SmartDashboard.putBoolean("loop running", true);
                    }

                    SmartDashboard.putBoolean("loop running", false);
                    TimeUnit.MILLISECONDS.sleep(500);
        
                    // While sensor is activated (reverse), drive storage motor
                    storageMotor.set(getMotorValue(acceptSpeed-0.1, acceptorFlipped));

                    while (!storageSensor.get() && !isStalled(storageMotor)) {
                        storageMotor.set(getMotorValue(acceptSpeed-0.1, storageFlipped));
                    }
                    
                    TimeUnit.MILLISECONDS.sleep(200);
                    storageMotor.set(0);

                    shootingDone = true;

                    Thread.currentThread().interrupt();
                } catch (Exception e) { shootingDone = true; Thread.currentThread().interrupt(); }
            }
        }).start();
    }

    public void shootTwice() {
        new Thread(new Runnable() {
            @Override
            public void run() {
                // shoot two times with a delay
                shootingDone = false;
                activateShooter(shootSpeed);
                while (!shootingDone) {}
                activateShooter(shootSpeed);
                shootingDone = true;
                Thread.currentThread().interrupt();
            }
        }).start();
    }

    public void checkBall() {
        // Everything should run seperately to prevent the updateSensors method to not work.

        // todo: make color senor more sensitive
        new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    if (proximity > PROXIMITY_THRESHOLD) {
                        if (color.blue > BLUE_THRESHOLD) {
                            switch (ballsLoaded) {
                                case 0: {
                                    acceptorMotor.set(getMotorValue(acceptSpeed, acceptorFlipped));
                                    storageMotor.set(getMotorValue(acceptSpeed, storageFlipped));

                                    // Add a delay before checking for stall current to reduce the possibility of the
                                    // extra power and low speed of starting the motor to activate the protection.
                                    TimeUnit.MILLISECONDS.sleep(100);

                                    while (storageSensor.get() && !isStalled(acceptorMotor) && !isStalled(storageMotor)) {
                                        acceptorMotor.set(getMotorValue(0.3, acceptorFlipped));
                                        storageMotor.set(getMotorValue(0.2, storageFlipped));
                                    }
                                }
                                case 1: {
                                    acceptorMotor.set(getMotorValue(acceptSpeed, acceptorFlipped));

                                    TimeUnit.MILLISECONDS.sleep(100);
                                
                                    while (acceptorSensor.get() && !isStalled(acceptorMotor)) {
                                        acceptorMotor.set(getMotorValue(0.3, acceptorFlipped));
                                    }
                                    TimeUnit.MILLISECONDS.sleep(250);
                                }
                                default: {}
                            }
                        
                            // This should run after the sensor loops are finished, stop the motors.
                            acceptorMotor.set(0);
                            storageMotor.set(0);

                        } else if (color.red > RED_THRESHOLD) {
                            acceptorMotor.set(getMotorValue((-acceptSpeed)-0.1, acceptorFlipped));
                            TimeUnit.MILLISECONDS.sleep(250);
                        } else {
                            acceptorMotor.set(0);
                        }
                    } else {
                        acceptorMotor.set(0);
                    }
                } catch (InterruptedException e) {
                    SmartDashboard.putBoolean("storage exception", true);
                    Thread.currentThread().interrupt();
                }
            }
        }).start();
    }

    private void resetErrors() {
        SmartDashboard.putBoolean("jammed", false);
        SmartDashboard.putBoolean("storage exception", false);
    }

    private void updateSensors() {
        color = colorSensor.getColor();
        proximity = colorSensor.getProximity();

        acceptorCurrent = acceptorMotor.getOutputCurrent();
        storageCurrent = storageMotor.getOutputCurrent();

        acceptorRPM = acceptorMotorEncoder.getVelocity();
        storageRPM = storageMotorEncoder.getVelocity();

        shooterRPM = shooterEncoder.getVelocity();
        shooterCurrent = shooterMotor.getOutputCurrent();

        // Values that we may need for testing
        SmartDashboard.putNumber("red", color.red);
        SmartDashboard.putNumber("green", color.green);
        SmartDashboard.putNumber("blue", color.blue);
        SmartDashboard.putNumber("proximity", proximity);

        SmartDashboard.putBoolean("acceptor loaded", !acceptorSensor.get());
        SmartDashboard.putBoolean("storage loaded", !storageSensor.get());

        SmartDashboard.putNumber("acceptor amps", acceptorCurrent);
        SmartDashboard.putNumber("storage amps", storageCurrent);

        SmartDashboard.putNumber("shooter rpm", shooterRPM);
        SmartDashboard.putNumber("shooter amps", shooterCurrent);

        SmartDashboard.putNumber("acceptor rpm", acceptorRPM);
        SmartDashboard.putNumber("storage rpm", storageRPM);

        SmartDashboard.putNumber("balls loaded", ballsLoaded);

        
        if (!storageSensor.get()) {
            // If the storage sensor (rear) is covered, we know there is a ball inside of it.
            ballsLoaded = 1;

            if (!acceptorSensor.get()) {
                // In addition, if the other sensor (front) is covered, there are two balls and can't accept more.
                ballsLoaded = 2;
            }
        } else {
            ballsLoaded = 0;
        }
    
    }

    @Override
    public void teleopInit() {
        SmartDashboard.putBoolean("storage exception", false);
        SmartDashboard.putBoolean("jammed", false);
        SmartDashboard.putBoolean("loop running", false);
    }

    @Override
    public void teleopPeriodic() {
        updateSensors();
        checkBall();

        if (joystick.getRawButtonPressed(SHOOT_JOY_PORT)) {
            activateShooter(shootSpeed);
        }

        if (joystick.getRawButtonPressed(RESET_JOY_PORT)) {
            resetErrors();
        }

        if (joystick.getRawButtonPressed(6)) {
            shootTwice();
        }
    }
}