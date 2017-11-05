package org.ftcTeam.opmodes.registrar1;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.ftcbootstrap.ActiveOpMode;

/**
 * Created by NovaLabs Robotics on 10/24/2017.
 */
@TeleOp
public class FRRTeleop extends ActiveOpMode {

        private DcMotor frontLeft;
        private DcMotor frontRight;
        private DcMotor backLeft;
        private DcMotor backRight;
        private DcMotor lift;
        private Servo servoLeft;
        private Servo servoRight; //clampLeft, clampRight, shoulder, elbow
        private Servo clampLeft;
        private Servo clampRight;
        private Servo shoulder;
        private Servo elbow;
        private boolean grabberClosed = true;
        private ColorSensor color;
        private OpticalDistanceSensor ods;

        private float x;
        private float y;
        private float z;

        private static float gY;

        public int position = 0;

        private static final float DEAD_ZONE = 0.2f;
        private static final double MAX_SPEED = 0.8d;

        private double[] motorPowers = new double[4];
        private static final int FRONT_LEFT  = 0;
        private static final int FRONT_RIGHT = 1;
        private static final int BACK_LEFT   = 2;
        private static final int BACK_RIGHT  = 3;

    // Lift Constants
        private static float UP_POWER = 0.8f;
        private static float DOWN_POWER = 0.2f;
        private static int GROUND_HEIGHT = 0;
        private static int ROW1_HEIGHT = 30;
        private static int ROW2_HEIGHT = 60;
        private static int ROW3_HEIGHT = 90;
        private static int MAX_HEIGHT = 120;

        private float desiredShoulder = 0.96f;
        private float desiredElbow = 0.91f;
        private static final float shoulderInc = .005f;
        private static final float elbowInc = .002f;

    @Override
    protected void onInit() {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clampLeft = hardwareMap.servo.get("clampLeft");
        clampRight = hardwareMap.servo.get("clampRight");
        shoulder = hardwareMap.servo.get("shoulder");
        elbow = hardwareMap.servo.get("elbow");

        lift = hardwareMap.dcMotor.get("lift");
        //lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setPower(0.0);

        shoulder.setPosition(desiredShoulder);
        elbow.setPosition(desiredElbow);
    }

    protected void activeLoop() throws InterruptedException {

        Gamepad driver = gamepad1;
        Gamepad gunner = gamepad2;
        updateJoyStickValues();

        boolean flipped = false;

        if (driver.y) {//drive foward; front is lift
            flipped = false;
        }
        else if (driver.a) {//drive backward; back is lift, front is relic grabber
            flipped = true;
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

        if (driver.right_bumper) {
            desiredShoulder += shoulderInc;
            shoulder.setPosition(desiredShoulder);
        }

        if (driver.left_bumper) {
            desiredShoulder -= shoulderInc;
            shoulder.setPosition(desiredShoulder);
        }

        if (driver.right_trigger > 0.05) {
            desiredElbow += elbowInc;
            elbow.setPosition(desiredElbow);
        }

        if (driver.left_trigger > 0.05) {
            desiredElbow -= elbowInc;
            elbow.setPosition(desiredElbow);
        }

        getTelemetryUtil().addData("Shoulder angle: ", desiredShoulder);
        getTelemetryUtil().addData("Elbow angle: ", desiredElbow);

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



                         //position 0 means lowest point, before block is picked up
                         //position 1 is height to put the bottom block on top of the first block, etc.
        lift.setPower(alterLiftPower());


        if (gunner.a) //lowest height/ground height; press 'A' to put lift at position 0
                      //target positions need to be tested!!!
        {
            lift.setPower(DOWN_POWER);
            lift.setTargetPosition(0);
            position = 0;
        }

        if (gunner.b) //second to lowest height, Row 1
        {
            if (position > 1) { // Above target height, move down

                lift.setPower(DOWN_POWER);
                lift.setTargetPosition(ROW1_HEIGHT);
            }
            else { // Below target height, move up

                lift.setPower(UP_POWER);
                lift.setTargetPosition(ROW1_HEIGHT);
            }
            position = 1;
        }
        if (gunner.y) //Row 2 height
        {

            lift.setPower(UP_POWER);
            lift.setTargetPosition(ROW2_HEIGHT);
        }
        if (gunner.x) //highest height
        {

            lift.setPower(UP_POWER);
            lift.setTargetPosition(ROW3_HEIGHT);
        }
        if (gunner.right_bumper)
        {
            if (!grabberClosed) {
                //servoLeft.setPower(.15);
                clampLeft.setPosition(.4);
                //servoRight.setPower(.15);
                clampRight.setPosition(.6);
                if(getTimer().targetReached(.25))
                    grabberClosed = true;
            } else {
                //servoLeft.setPower(.15);
                clampLeft.setPosition(0.70);
                //clampLeft.setPosition(0.75);
                //servoRight.setPower(.15);
                clampRight.setPosition(0.30);
                //clampRight.setPosition(0.25);
                if(getTimer().targetReached(.25))
                    grabberClosed = false;
            }
        }

        getTelemetryUtil().sendTelemetry();
    }

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

        gY = gamepad2.left_stick_y;
       // gY = adjustForDeadZone(gY);
       // gY = shapeInput(gY);
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
                shapedValue = input * input * input;
                //shapedValue = input * -input;
            }
            else {
                shapedValue = input * input * input;
                //shapedValue = input * input;
            }
        }

        return shapedValue;

        // return input * (input < 0.0f ? -input : input);
    }


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

    private static float alterLiftPower() {
        float ogY = -gY;

        if (ogY<0) {
            return ogY*.15f;
        } else {
            return ogY*.85f;
        }

    }

    }
