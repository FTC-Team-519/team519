package org.ftcTeam.opmodes.registrar1;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

import org.ftcbootstrap.ActiveOpMode;
import org.ftcbootstrap.components.TimerComponent;

/**
 * Created by NovaLabs Robotics on 1/8/2017.
 */

public class TeleopUltrasonicTest extends ActiveOpMode{
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private I2cDevice ultrasonicSensor;
    public I2cDeviceSynch ultrasonicReader;

    byte[] ultrasonicCache;
    public static final int ULTRASONIC_REG_START = 0x04; //Register to start reading
    public static final int ULTRASONIC_READ_LENGTH = 2; //Number of byte to read


    private float x;
    private float y;
    private float z;


    private double[] motorPowers = new double[4];
    private static final int FRONT_LEFT  = 0;
    private static final int FRONT_RIGHT = 1;
    private static final int BACK_LEFT   = 2;
    private static final int BACK_RIGHT  = 3;

    private static final float DEAD_ZONE = 0.2f;
    private static final double MAX_SPEED = 1.0d;

    TimerComponent timerComponent;

    boolean flipped = false;

    /**
     * Implement this method to define the code to run when the Init button is pressed on the Driver station.
     */
    @Override
    protected void onInit() {

        //Note The Telemetry Utility is designed to let you organize all telemetry data before sending it to
        //the Driver station via the sendTelemetry command
        getTelemetryUtil().addData("Init", getClass().getSimpleName() + " onInit.");
        getTelemetryUtil().sendTelemetry();

        frontLeft = hardwareMap.dcMotor.get("motor4");
        frontRight = hardwareMap.dcMotor.get("motor2");
        backLeft = hardwareMap.dcMotor.get("motor3");
        backRight = hardwareMap.dcMotor.get("motor1");
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        I2cAddr ultrasonicAddress = new I2cAddr(0x14);
        ultrasonicSensor = hardwareMap.i2cDevice.get("range");
        ultrasonicReader = new I2cDeviceSynchImpl(ultrasonicSensor, ultrasonicAddress, false);
        ultrasonicReader.engage();

    }

    @Override
    protected void onStart() throws InterruptedException {
        super.onStart();

        getTelemetryUtil().addData("Start", getClass().getSimpleName() + " onStart.");
        getTelemetryUtil().sendTelemetry();

        timerComponent = getTimer();
    }

    /**
     * Implement this method to define the code to run when the Start button is pressed on the Driver station.
     * This method will be called on each hardware cycle just as the loop() method is called for event based Opmodes
     *
     * @throws InterruptedException
     */
    @Override
    protected void activeLoop() throws InterruptedException {
        Gamepad driver = gamepad1;

        updateJoyStickValues();

        if (driver.y) {
            flipped = false;
        }
        else if (driver.a) {
            flipped = true;
        }
        if (driver.x) {
            if (ultrasonicCache[0] > 15) {
                frontLeft.setPower(-1.0);
                backLeft.setPower(-1.0);
                frontRight.setPower(-1.0);
                backRight.setPower(-1.0);
            }
            else {
                frontLeft.setPower(0);
                backLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(0);
            }
        }

        if (flipped) {
            x = -x;
            y = -y;
        }

        if (Math.abs(driver.left_stick_x)>Math.abs(driver.left_stick_y)){
            y = 0;
        }
        else if (Math.abs(driver.left_stick_y) > Math.abs(driver.left_stick_x)){
            x = 0;
        }
        else if (driver.x) {
            x = -100;
            y = 0;
        }
        else if (driver.b) {
            x = 100;
            y = 0;
        }

        // Forward/backward power is left_stick_y, but forward is -1.0 reading, so invert
        double pwr = -y;

        motorPowers[FRONT_RIGHT] = pwr - x - z;
        motorPowers[FRONT_LEFT] = 1.25*(pwr + x + z);
        motorPowers[BACK_RIGHT] = 1.25*(pwr + x - z);
        motorPowers[BACK_LEFT] = pwr - x + z;
        normalizeCombinedPowers(motorPowers);

        frontRight.setPower(reducePower(motorPowers[FRONT_RIGHT]));
        frontLeft.setPower(reducePower(motorPowers[FRONT_LEFT]));
        backRight.setPower(reducePower(motorPowers[BACK_RIGHT]));
        backLeft.setPower(reducePower(motorPowers[BACK_LEFT]));

        ultrasonicCache = ultrasonicReader.read(ULTRASONIC_REG_START, ULTRASONIC_READ_LENGTH);

        telemetry.addData("Ultra Sonic", ultrasonicCache[0] & 0xFF);
        telemetry.addData("ODS", ultrasonicCache[1] & 0xFF);
        getTelemetryUtil().sendTelemetry();

        // Might want to have a more effective combination
        //frontRight.setPower(Range.clip(pwr - x - z, -MAX_SPEED, MAX_SPEED));
        //frontLeft.setPower(Range.clip(pwr + x + z, -MAX_SPEED, MAX_SPEED));
        //backRight.setPower(Range.clip(pwr + x - z, -MAX_SPEED, MAX_SPEED));
        //backLeft.setPower(Range.clip(pwr - x + z, -MAX_SPEED, MAX_SPEED));
    }

    /**
     * Returns power values normalized across motors so that they are all within the
     * expected [-1.0 .. 1.0] range, which is often not true after combining values for mecanum.
     *
     * @param motorPowers the motor values to normalize to a max point.
     */
    private static void normalizeCombinedPowers(double[] motorPowers) {
        double maxAbsPower = 0.0d;

        for (double motorPower : motorPowers) {
            double tmpAbsPower = Math.abs(motorPower);
            if (tmpAbsPower > maxAbsPower) {
                maxAbsPower = tmpAbsPower;
            }
        }

        if (maxAbsPower > 1.0d) {
            for (int i = 0; i < motorPowers.length; ++i) {
                motorPowers[i] = motorPowers[i] / maxAbsPower;
            }
        }
    }

    /**
     * Returns a power that is reduced proportionally to the maximum speed, which would be
     * no reduction if max speed is 1.0 (or basically 100%).
     *
     * @param input power to potentially be reduced.
     * @return a power that is reduced proportionally to the maximum speed, which would be
     * no reduction if max speed is 1.0 (or basically 100%).
     */
    private static double reducePower(double input) {
        // return input;   // If no reduction desired
        return input * MAX_SPEED;
    }

    /**
     * Re-reads joystick values and assigns the state variables used for other computations.
     */
    private void updateJoyStickValues() {
        y = gamepad1.left_stick_y;
        x = gamepad1.left_stick_x;
        z = gamepad1.right_stick_x;

        // NOTE: Check for deadzone before any shaping
        y = adjustForDeadZone(y);
        x = adjustForDeadZone(x);
        z = adjustForDeadZone(z);

        y = shapeInput(y);
        x = shapeInput(x);
        z = shapeInput(z);
    }

    /**
     * Returns 0.0 if within the deadzone range, otherwise the original value.
     *
     * @param input value to check.
     * @return 0.0 if within the deadzone range, otherwise the original value.
     */
    private static float adjustForDeadZone(float input) {
        float adjustedValue = input;

        if (Math.abs(input) < DEAD_ZONE) {
            adjustedValue = 0.0f;
        }

        return adjustedValue;
    }

    /**
     * Returns the input value shaped by squaring and preserving sign.
     *
     * REVISIT: This is where using a more custom curve could be done instead.
     *
     * @param input value to shape.
     * @return the input value shaped by squaring and preserving sign.
     */
    private static float shapeInput(float input) {
        float shapedValue = 0.0f;
        if (input != 0.0f) {
            if (input < 0.0f) {
                shapedValue = input * -input;
            }
            else {
                shapedValue = input * input;
            }
        }

        return shapedValue;

        // return input * (input < 0.0f ? -input : input);
    }
}
