package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class Robot extends TimedRobot {
    private static final I2C.Port COLOR_SENSOR_PORT = I2C.Port.kOnboard;

    // TalonSRX colorMotor = new TalonSRX(2);
    CANSparkMax colorMotor = new CANSparkMax(8, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax intakeMotor = new CANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless);
    DigitalInput photoElectric = new DigitalInput(0);


    private ColorSensorV3 colorSensor = new ColorSensorV3(COLOR_SENSOR_PORT);

    private double baseRed, baseGreen, baseBlue, currentRed, currentGreen, currentBlue;
    private int proximity;


    @Override
    public void robotInit() {
    }

    @Override
    public void simulationInit() {

    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void teleopInit() {
      
    }

    @Override
    public void testInit() {

    }

    @Override
    public void robotPeriodic() {

    }

    @Override
    public void simulationPeriodic() {

    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopPeriodic() {
        Color detectedColor = colorSensor.getColor();

        SmartDashboard.putNumber("red", detectedColor.red);
        SmartDashboard.putNumber("green", detectedColor.green);
        SmartDashboard.putNumber("blue", detectedColor.blue);
        SmartDashboard.putNumber("ir", colorSensor.getIR());

       // Color detectedColor = m_colorSensor.getColor();

        double IR = colorSensor.getIR();
          
        SmartDashboard.putNumber("Red", currentRed);
        SmartDashboard.putNumber("Green", currentGreen);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("IR", IR);     
      
        SmartDashboard.putNumber("Proximity", proximity);

        SmartDashboard.putBoolean("tripped", photoElectric.get());

        currentRed = detectedColor.red;
        currentGreen = detectedColor.green;
        currentBlue = detectedColor.blue;
        proximity = colorSensor.getProximity();
        
        //if(colorSensor.getProximity() > 500){
        //  colorMotor.set(0.5);
        //}
        //else{
        //  colorMotor.set(0);
        //}

        // red ball =
        // red: 0.53
        // green: 0.35
        // blue: 0.11
        // ir: 35
        // proximity: 676

        // blue ball = 
        // red: 0.15
        // blue: 0.43 max, 0.25 starting
        // green: 0.40
        // proximity: 760

        if (proximity > 120) {
          if (currentBlue > 0.35) {
            // blue ball is starting to enter, accept and keep motor running
            colorMotor.set(-0.3);

            // run other motor
            intakeMotor.set(0.3);

            while (photoElectric.get()) {}

            intakeMotor.set(0);
            colorMotor.set(0);
          } else if (currentRed > 0.35) {
            // red ball is starting to enter, reject and reverse motor
            colorMotor.set(0.3);
          } else {
            colorMotor.set(0);
          }
        } else {
          colorMotor.set(0);
        }
      
    }

    // @Override
    // public void teleopPeriodic() {
    //     double x = xyStick.getX();
    //     double y = xyStick.getY();
    //     double z = zStick.getX();

    //     Rotation2d rotation = gyro.getRotation2d();

    //     SmartDashboard.putNumber("X Input", x);
    //     SmartDashboard.putNumber("Y Input", y);
    //     SmartDashboard.putNumber("Z Input", z);
    //     SmartDashboard.putNumber("Gyro Angle", rotation.getDegrees());

    //     ChassisSpeeds speeds =
    //             ChassisSpeeds.fromFieldRelativeSpeeds(x, y, z, rotation);

    //     chassis.drive(speeds);
    // }

    @Override
    public void testPeriodic() {

    }

////////////////////////////////


}