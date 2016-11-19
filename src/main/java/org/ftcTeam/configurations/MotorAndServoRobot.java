package org.ftcTeam.configurations;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.ftcbootstrap.RobotConfiguration;
import org.ftcbootstrap.components.utils.TelemetryUtil;

/**
 * MyFirstBot Saved Configuration
 * <p/>
 * It is assumed that there is a configuration on the phone running the Robot Controller App with the same name as this class and
 * that  configuration is the one that is marked 'activated' on the phone.
 * It is also assumed that the device names in the 'init()' method below are the same  as the devices named for the
 * saved configuration on the phone.
 */
public class MotorAndServoRobot extends RobotConfiguration {

    //motors
    public DcMotor motor1;
    public DcMotor motor2;
    public DcMotor motor3;
    public DcMotor motor4;
    public IrSeekerSensor IRSeeker;
    public Servo servo1;

    /**
     * Assign your class instance variables to the saved device names in the hardware map
     *
     * @param hardwareMap
     * @param telemetryUtil
     */
    @Override
    protected void init(HardwareMap hardwareMap, TelemetryUtil telemetryUtil) {

        setTelemetry(telemetryUtil);

        servo1 = (Servo) getHardwareOn("servo1", hardwareMap.servo);
        motor1 = (DcMotor) getHardwareOn("motor1", hardwareMap.dcMotor);
        motor2 = (DcMotor) getHardwareOn("motor2", hardwareMap.dcMotor);
        motor3 = (DcMotor) getHardwareOn("motor3", hardwareMap.dcMotor);
        motor4 = (DcMotor) getHardwareOn("motor4", hardwareMap.dcMotor);
        IRSeeker = (IrSeekerSensor) getHardwareOn("irseeker", hardwareMap.irSeekerSensor);
    }


    /**
     * Factory method for this class
     *
     * @param hardwareMap
     * @param telemetryUtil
     * @return
     */
    public static MotorAndServoRobot newConfig(HardwareMap hardwareMap, TelemetryUtil telemetryUtil) {

        MotorAndServoRobot config = new MotorAndServoRobot();
        config.init(hardwareMap, telemetryUtil);
        return config;

    }



}
