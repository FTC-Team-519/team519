package org.ftcTeam.opmodes.registrar1;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.ftcbootstrap.ActiveOpMode;
import org.ftcbootstrap.components.operations.motors.GamePadTankDrive;


/**
 * Note:  It is assumed that the proper registry is used for this set of demos. To confirm please
 * search for "Enter your custom registry here"  in  {@link org.ftcTeam.FTCTeamControllerActivity}
 * <p/>
 * Summary:  Use an Operation class to perform a tank drive using the gamepad joysticks.
 * See: {@link GamePadTankDrive}
 */

@Autonomous
@Disabled
public class AutonomousFirstTry extends ActiveOpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private double fL = 0.0d;
    private double fR = 0.0d;
    private double bL = 0.0d;
    private double bR = 0.0d;

    private int sillyCounter = 0;

    private double[] motorPowers = new double[4];
    private static final int FRONT_LEFT  = 0;
    private static final int FRONT_RIGHT = 1;
    private static final int BACK_LEFT   = 2;
    private static final int BACK_RIGHT  = 3;

    private static final float DEAD_ZONE = 0.2f;
    private static final double MAX_SPEED = 0.4d;

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
    }

    @Override
    protected void onStart() throws InterruptedException {
        super.onStart();

        getTelemetryUtil().addData("Start", getClass().getSimpleName() + " onStart.");
        getTelemetryUtil().sendTelemetry();
    }

    /**
     * Implement this method to define the code to run when the Start button is pressed on the Driver station.
     * This method will be called on each hardware cycle just as the loop() method is called for event based Opmodes
     *
     * @throws InterruptedException
     */
    @Override
    protected void activeLoop() throws InterruptedException {

        long currTime = System.currentTimeMillis();
        if (getTimer().targetReached(3.0d)) {
            getTelemetryUtil().addData("Found", "Reached 3 seconds");
            getTelemetryUtil().sendTelemetry();
        }

        fR += 0.01d;
        fL += 0.02d;
        bR += 0.03d;
        bL += 0.04d;

        frontRight.setPower(fR);
        frontLeft.setPower(fL);
        backRight.setPower(bR);
        backLeft.setPower(bL);

        if (sillyCounter > 25) {
            sillyCounter = 0;
            /**
            getTelemetryUtil().addData("Start", "fR: " + frontRight.getPower() +
                    ", fL: " + frontLeft.getPower() +
                    ", bR: " + backRight.getPower() +
                    ", bL: " + backLeft.getPower());

            getTelemetryUtil().addData("Other", "fR: " + fR +
                    ", fL: " + fL +
                    ", bR: " + bR +
                    ", bL: " + bL);

            getTelemetryUtil().sendTelemetry();
             **/
        }
        else {
            ++sillyCounter;
        }
   }

}
