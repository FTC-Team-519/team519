package org.ftcTeam.opmodes.registrar1;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.ftcbootstrap.ActiveOpMode;

/**
 * Created by NovaLabs Robotics on 10/24/2017.
 */
@TeleOp
public class FRRTeleop extends ActiveOpMode {

        private DcMotor shooter;
        private DcMotor frontLeft;
        private DcMotor frontRight;
        private DcMotor backLeft;
        private DcMotor backRight;
        private DcMotor midCollector;
        private DcMotor frontCollector;
        private DcMotor topCollector;
        private DcMotor lift;
        private Servo servoLeft;
        private Servo servoRight;
        private boolean grabberClosed;
        private ColorSensor color;
        private OpticalDistanceSensor ods;
        private float x;
        private float y;
        private float z;

        private double[] motorPowers = new double[4];
        private static final int FRONT_LEFT  = 0;
        private static final int FRONT_RIGHT = 1;
        private static final int BACK_LEFT   = 2;
        private static final int BACK_RIGHT  = 3;

    @Override
    protected void onInit() {
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(0.0);
    }

    protected void activeLoop() throws InterruptedException {

        Gamepad driver = gamepad1;
        Gamepad gunner = gamepad2;

        //updateJoyStickValues();


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

        // Forward/backward power is left_stick_y, but forward is -1.0 reading, so invert
        double pwr = -y;

        motorPowers[FRONT_RIGHT] = pwr - x - z;
        motorPowers[FRONT_LEFT] = 1.25*(pwr + x + z);
        motorPowers[BACK_RIGHT] = 1.25*(pwr + x - z);
        motorPowers[BACK_LEFT] = pwr - x + z;
        //normalizeCombinedPowers(motorPowers);

        //frontRight.setPower(reducePower(motorPowers[FRONT_RIGHT]));
        //frontLeft.setPower(reducePower(motorPowers[FRONT_LEFT]));
        //backRight.setPower(reducePower(motorPowers[BACK_RIGHT]));
        //backLeft.setPower(reducePower(motorPowers[BACK_LEFT]));
        int position = 0;//position 0 means lowest point, before block is picked up
                         //position 1 is height to put the bottom block on top of the first block, etc.
        if (gunner.a) //lowest height/ground height; press a to put lift at position 0
                      //target positions need to be tested!!!
        {
            lift.setPower(.15);
            lift.setTargetPosition(10);
        }

        if (gunner.b) //second lowest height
        {

            lift.setPower(.15);
            lift.setTargetPosition(20);
        }
        if (gunner.y) //third lowest height
        {

            lift.setPower(.15);
            lift.setTargetPosition(30);
        }
        if (gunner.x) //highest height
        {

            lift.setPower(.15);
            lift.setTargetPosition(40);
        }
        if (gunner.right_bumper)
        {
            if (!grabberClosed) {
                //servoLeft.setPower(.15);
                servoLeft.setPosition(.5);
                //servoRight.setPower(.15);
                servoRight.setPosition(.5);
                grabberClosed = true;
            } else {
                //servoLeft.setPower(.15);
                servoLeft.setPosition(0.25);
                //servoRight.setPower(.15);
                servoRight.setPosition(0.75);
                grabberClosed = false;
            }
        }





    }



    }
