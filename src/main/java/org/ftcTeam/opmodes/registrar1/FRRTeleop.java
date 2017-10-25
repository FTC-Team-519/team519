package org.ftcTeam.opmodes.registrar1;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.ftcbootstrap.ActiveOpMode;

/**
 * Created by NovaLabs Robotics on 10/24/2017.
 */

public class FRRTeleop {
    @TeleOp
        private DcMotor shooter;
        private DcMotor frontLeft;
        private DcMotor frontRight;
        private DcMotor backLeft;
        private DcMotor backRight;
        private DcMotor midCollector;
        private DcMotor frontCollector;
        private DcMotor topCollector;
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

    protected void activeLoop() throws InterruptedException {
        Gamepad driver = gamepad1;
        Gamepad gunner = gamepad2;

        updateJoyStickValues();

        if (driver.y) {
            flipped = false;
        }
        else if (driver.a) {
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
        normalizeCombinedPowers(motorPowers);

        frontRight.setPower(reducePower(motorPowers[FRONT_RIGHT]));
        frontLeft.setPower(reducePower(motorPowers[FRONT_LEFT]));
        backRight.setPower(reducePower(motorPowers[BACK_RIGHT]));
        backLeft.setPower(reducePower(motorPowers[BACK_LEFT]));

    }
