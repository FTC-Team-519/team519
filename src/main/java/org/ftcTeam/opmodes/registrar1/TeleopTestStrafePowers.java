package org.ftcTeam.opmodes.registrar1;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.ftcbootstrap.ActiveOpMode;
import org.ftcbootstrap.components.TimerComponent;

public class TeleopTestStrafePowers extends ActiveOpMode {
    private DcMotor shooter;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor midCollector;
    private DcMotor frontCollector;
    private ColorSensor color;
    private OpticalDistanceSensor ods;
    private float x;
    private float y;
    private float z;

    private int sillyCounter = 0;

    private double[] motorPowers = new double[4];
    private static final int FRONT_LEFT  = 0;
    private static final int FRONT_RIGHT = 1;
    private static final int BACK_LEFT   = 2;
    private static final int BACK_RIGHT  = 3;

    private static final float DEAD_ZONE = 0.2f;
    private static final double MAX_SPEED = 1.0d;

    TimerComponent timerComponent;

    int previousTickCount = 0;

    boolean keepShooterSpinning = false;
    double SHOOTER_REVERSE_SPEED = -0.1d;
    double SHOOTER_FORWARD_SPEED = 1.0d;
    double currentShooterSpeed = 0.0d;

    boolean flipped = false;

    double lf_power_adjust;
    double lr_power_adjust;
    double rf_power_adjust;
    double rr_power_adjust;

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

        midCollector = hardwareMap.dcMotor.get("feeder");
        frontCollector = hardwareMap.dcMotor.get("collector");
        shooter = hardwareMap.dcMotor.get("shooter");

        color = hardwareMap.colorSensor.get("colorSensor1");
        color.enableLed(false);

        lf_power_adjust = 0.0;
        lr_power_adjust = 0.0;
        rf_power_adjust = 0.0;
        rr_power_adjust = 0.0;
    }

    @Override
    protected void onStart() throws InterruptedException {
        super.onStart();

        getTelemetryUtil().addData("Start", getClass().getSimpleName() + " onStart.");
        getTelemetryUtil().sendTelemetry();

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.getController().setMotorMode(1, DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.getController().setMotorZeroPowerBehavior(1, DcMotor.ZeroPowerBehavior.FLOAT);
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
        Gamepad gunner = gamepad2;

        updateJoyStickValues();

        if (Math.abs(driver.left_stick_x)>Math.abs(driver.left_stick_y)){
            y = 0;
        }
        else if (Math.abs(driver.left_stick_y) > Math.abs(driver.left_stick_x)){
            x = 0;
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

        if (sillyCounter > 25) {
            sillyCounter = 0;
            getTelemetryUtil().addData("Start", "fR: " + frontRight.getPower() +
                    ", fL: " + frontLeft.getPower() +
                    ", bR: " + backRight.getPower() +
                    ", bL: " + backLeft.getPower());
            getTelemetryUtil().sendTelemetry();
        }
        else {
            ++sillyCounter;
        }

        // Forward/backward power is left_stick_y, but forward is -1.0 reading, so invert

//        if (gamepad1.x) {
//            currPower = 0.1d;
//        }
//        else if (gamepad1.y) {
//            currPower = 0.2d;
//        }
//        else if (gamepad1.b) {
//            currPower = 0.4d;
//        }
//        else if (gamepad1.a) {
//            currPower = 0.6d;
//        }
//        else if (gamepad1.left_bumper) {
//            currPower = 0.8d;
//        }
//        else if (gamepad1.right_bumper) {
//            currPower = 1.0d;
//        }

        if (gunner.x || gunner.a || gunner.b) {
            if (gunner.x) {
                currentShooterSpeed = SHOOTER_REVERSE_SPEED;
                keepShooterSpinning = false;
            }
            else if (gunner.b) {
                currentShooterSpeed = SHOOTER_FORWARD_SPEED;
                keepShooterSpinning = true;
            }
            else if (gunner.a) {
                currentShooterSpeed = 0.0d;
                keepShooterSpinning = false;
            }
        }
        else {
            if (! keepShooterSpinning) {
                currentShooterSpeed = 0.0d;
            }
            /** Driver potentially saved controls
            if (driver.x) {
                currPower = -1.0d;
            } else if (driver.b) {
                currPower = 1.0d;
            }
             **/
        }

        // Negative, as the wheel needs to go in reverse direction (could reverse motor actually)
        shooter.setPower(currentShooterSpeed);

        // Gunner

        if (gunner.right_bumper) {
            midCollector.setPower(0.5);
        }
        else if (gunner.left_bumper) {
            midCollector.setPower(-0.5);
        }
        else {
            midCollector.setPower(0.0);
        }

        // Driver
        if (driver.right_bumper) {
            frontCollector.setPower(-0.5);
        }
        else if (driver.left_bumper) {
            frontCollector.setPower(0.5);
        }
        else if ((driver.left_trigger > 0) || (driver.right_trigger > 0)){
            frontCollector.setPower(0.0);
        }

        getTelemetryUtil().addData("RGB: ", "" + color.red() + "," + color.green() + "," + color.blue());
        getTelemetryUtil().addData("ARGB", " " + color.argb());

        if (timerComponent.targetReached(1.0f)) {
            int currentTicks = shooter.getController().getMotorCurrentPosition(1);
            float ticksPerSecond = currentTicks - previousTickCount;
            previousTickCount = currentTicks;
            getTelemetryUtil().addData("RPM", "ticksPerMinute: " + ticksPerSecond);
        }

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
