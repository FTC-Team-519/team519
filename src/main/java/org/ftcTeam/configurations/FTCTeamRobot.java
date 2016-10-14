package org.ftcTeam.configurations;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.ftcbootstrap.RobotConfiguration;
import org.ftcbootstrap.components.utils.TelemetryUtil;

/**
 * FTCTeamRobot Saved Configuration
 * <p/>
 * It is assumed that there is a configuration on the phone running the Robot Controller App with the same name as this class and
 * that  configuration is the one that is marked 'activated' on the phone.
 * It is also assumed that the device names in the 'init()' method below are the same  as the devices named for the
 * saved configuration on the phone.
 */
public class FTCTeamRobot extends RobotConfiguration {

    //motors
    public DcMotor motor1;
    public DcMotor motor2;
    public DcMotor motor3;
    public DcMotor motor4;
    public Servo servo;

    /**
     * Factory method for this class
     *
     * @param hardwareMap the hardware map to read from.
     * @param telemetryUtil the telemetryUtil to potentially write out to.
     * @return a new FTCTeamRobot.
     */
    public static FTCTeamRobot newConfig(HardwareMap hardwareMap, TelemetryUtil telemetryUtil) {

        FTCTeamRobot config = new FTCTeamRobot();
        config.init(hardwareMap, telemetryUtil);
        return config;

    }

    /**
     * Assign your class instance variables to the saved device names in the hardware map
     *
     * @param hardwareMap the hardware map to read from.
     * @param telemetryUtil the telemetryUtil to potentially write out to.
     */
    @Override
    protected void init(HardwareMap hardwareMap, TelemetryUtil telemetryUtil) {

        setTelemetry(telemetryUtil);

        //touch = (TouchSensor) getHardwareOn("touch", hardwareMap.touchSensor);
        //servo = (Servo) getHardwareOn("servo", hardwareMap.servo);
        motor1 = (DcMotor) getHardwareOn("motor1", hardwareMap.dcMotor);
        motor2 = (DcMotor) getHardwareOn("motor2", hardwareMap.dcMotor);
        motor1.setDirection(DcMotor.Direction.REVERSE);

        motor3 = (DcMotor) getHardwareOn("motor3", hardwareMap.dcMotor);
        motor4 = (DcMotor) getHardwareOn("motor4", hardwareMap.dcMotor);
        motor3.setDirection(DcMotor.Direction.REVERSE);


    }




}
