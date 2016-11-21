package org.ftcTeam.opmodes.registrar1;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.ftcbootstrap.ActiveOpMode;
import org.ftcbootstrap.components.TimerComponent;
import org.ftcbootstrap.components.operations.motors.GamePadTankDrive;


/**
 * Note:  It is assumed that the proper registry is used for this set of demos. To confirm please
 * search for "Enter your custom registry here"  in  {@link org.ftcTeam.FTCTeamControllerActivity}
 * <p/>
 * Summary:  Use an Operation class to perform a tank drive using the gamepad joysticks.
 * See: {@link GamePadTankDrive}
 */

@TeleOp
public class TestShooterSpeed extends ActiveOpMode {
    private DcMotor shooter;
    float x;
    float y;
    float z;
    private static final float DEAD_ZONE = 0.2f;
    TimerComponent timerComponent;

    int previousTickCount = 0;
    float voltageTarget = 0.0f;
    VoltageSensor voltageSensor;

    /**
     * Implement this method to define the code to run when the Init button is pressed on the Driver station.
     */
    @Override
    protected void onInit() {

        //Note The Telemetry Utility is designed to let you organize all telemetry data before sending it to
        //the Driver station via the sendTelemetry command
        getTelemetryUtil().addData("Init", getClass().getSimpleName() + " onInit.");
        getTelemetryUtil().sendTelemetry();

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        shooter = hardwareMap.dcMotor.get("shooter");
    }

    @Override
    protected void onStart() throws InterruptedException {
        super.onStart();

        getTelemetryUtil().addData("Start", getClass().getSimpleName() + " onStart.");
        getTelemetryUtil().sendTelemetry();

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        //shooter.getController().setMotorMode(1, DcMotor.RunMode.RUN_USING_ENCODER);
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
        updateJoyStickValues();

        // Forward/backward power is left_stick_y, but forward is -1.0 reading, so invert
        double pwr = -y;
        double currPower = 0.0d;

        if (gamepad1.right_bumper) {
            if (voltageTarget <= 1.0f) {
                voltageTarget += 0.01f;
            }
            currPower = voltageTarget;
        }
        else if (gamepad1.left_bumper) {
            if (voltageTarget >= 0.0f) {
                voltageTarget -= 0.01f;
            }
            currPower = voltageTarget;
        }
        if (gamepad1.x) {
            currPower = 0.4d;
        }
        else if (gamepad1.y) {
            currPower = 0.5d;
        }
        else if (gamepad1.b) {
            currPower = 0.6d;
        }
        else if (gamepad1.a) {
            currPower = 0.7d;
        }
        else if (gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_right) {
            currPower = voltageTarget;
        }

        // Negative, as the wheel needs to go in reverse direction (could reverse motor actually)
        shooter.setPower(currPower);

        if (timerComponent.targetReached(1.0f)) {
            int currentTicks = shooter.getController().getMotorCurrentPosition(1);
            float ticksPerSecond = currentTicks - previousTickCount;
            previousTickCount = currentTicks;
            getTelemetryUtil().addData("RPM", "ticksPerMinute: " + ticksPerSecond);
        }

        getTelemetryUtil().addData("Battery: ", "voltage: " + voltageSensor.getVoltage());
        getTelemetryUtil().addData("Speed:", "percent: " + currPower);
        getTelemetryUtil().sendTelemetry();
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
