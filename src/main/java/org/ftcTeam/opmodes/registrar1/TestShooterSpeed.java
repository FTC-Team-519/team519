package org.ftcTeam.opmodes.registrar1;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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

    /**
     * Implement this method to define the code to run when the Init button is pressed on the Driver station.
     */
    @Override
    protected void onInit() {

        //Note The Telemetry Utility is designed to let you organize all telemetry data before sending it to
        //the Driver station via the sendTelemetry command
        getTelemetryUtil().addData("Init", getClass().getSimpleName() + " onInit.");
        getTelemetryUtil().sendTelemetry();

        shooter = hardwareMap.dcMotor.get("shooter");
    }

    @Override
    protected void onStart() throws InterruptedException {
        super.onStart();

        getTelemetryUtil().addData("Start", getClass().getSimpleName() + " onStart.");
        getTelemetryUtil().sendTelemetry();

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
        updateJoyStickValues();

        // Forward/backward power is left_stick_y, but forward is -1.0 reading, so invert
        double pwr = -y;

        if (gamepad1.x) {
            shooter.setPower(0.10d);
        }
        else if (gamepad1.y) {
            shooter.setPower(0.20d);
        }
        else if (gamepad1.b) {
            shooter.setPower(0.40d);
        }
        else if (gamepad1.a) {
            shooter.setPower(0.6d);
        }
        else {
            shooter.setPower(0.0d);
        }

        if (timerComponent.targetReached(1.0f)) {
            int currentTicks = shooter.getController().getMotorCurrentPosition(1);
            float ticksPerSecond = currentTicks - previousTickCount;
            previousTickCount = currentTicks;
            getTelemetryUtil().addData("RPM", "ticksPerMinute: " + ticksPerSecond);
        }

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
