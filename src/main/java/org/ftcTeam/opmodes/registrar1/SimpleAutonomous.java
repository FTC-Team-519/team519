package org.ftcTeam.opmodes.registrar1;


import android.util.Base64;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.ftcbootstrap.ActiveOpMode;

import java.io.UnsupportedEncodingException;
import java.text.DecimalFormat;

/**
 * Autonomous Vuforia reference class that doesn't depend upon any motors, just the camera.
 */

@Autonomous
public class SimpleAutonomous extends ActiveOpMode {

    int step = -1;

    private DcMotor shooter;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor midCollector;
    private DcMotor topCollector;
    private DcMotor frontCollector;
    private ColorSensor colorSensor;

    private double[] motorPowers = new double[4];
    private static final int FRONT_LEFT  = 0;
    private static final int FRONT_RIGHT = 1;
    private static final int BACK_LEFT   = 2;
    private static final int BACK_RIGHT  = 3;

    boolean beaconIsBlue = false;

    private static final double MAX_SPEED = 0.3d;



    /**
     * Implement this method to define the code to run when the Init button is pressed on the Driver station.
     */
    @Override
    protected void onInit() {
        // FIXME: This is where we need to get the mapping correct!!!!
        // NOTE: For "forward bumper":  [FL: motor4, FR: motor2, BL: motor3, BR: motor1]
        // NOTE: For "backward bumper": [FL: motor1, FR: motor3, BL: motor2, BR: motor4]
        frontLeft = hardwareMap.dcMotor.get("motor4");
        frontRight = hardwareMap.dcMotor.get("motor2");
        backLeft = hardwareMap.dcMotor.get("motor3");
        backRight = hardwareMap.dcMotor.get("motor1");
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        midCollector = hardwareMap.dcMotor.get("feeder");
        frontCollector = hardwareMap.dcMotor.get("collector");
        topCollector = hardwareMap.dcMotor.get("topCollector");
        shooter = hardwareMap.dcMotor.get("shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.getController().setMotorZeroPowerBehavior(1, DcMotor.ZeroPowerBehavior.FLOAT);


        //Note The Telemetry Utility is designed to let you organize all telemetry data before sending it to
        //the Driver station via the sendTelemetry command
        getTelemetryUtil().addData("Init", getClass().getSimpleName() + " onInit.");
        getTelemetryUtil().sendTelemetry();
    }

    @Override
    protected void onStart() throws InterruptedException {
        super.onStart();

        // NOTE: Ensure motors not going anywhere at start
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);

        // NOTE: Start monitoring for images on the field

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
        boolean isVisible = false;
        getTelemetryUtil().addData("Step ", "" + step);

        switch(step) {
            case -1://wait
                if (getTimer().targetReached(7.0d)) {
                    ++step;
                }
                break;
            case 0://spin up shooter
                shooter.setPower(1.0d);
                if (getTimer().targetReached(2.5d)) {
                    ++step;
                }
                break;
            case 1://shoot
                midCollector.setPower(0.5d);
                topCollector.setPower(0.5d);
                frontCollector.setPower(0.5d);
                if (getTimer().targetReached(2.0d)) {
                    midCollector.setPower(0.0d);
                    topCollector.setPower(0.0d);
                    frontCollector.setPower(0.0d);
                    shooter.setPower(0.0d);
                    ++step;
                }
                break;
            case 2://park on center vortex
                forward(0.5d);
                if (getTimer().targetReached(1.7d)) {
                    stopMoving();
                    ++step;
                }
                break;
        }

        getTelemetryUtil().addData("Seen: ", isVisible ? "Visible" : "Not Visible");
        getTelemetryUtil().sendTelemetry();
    }
    public void forward (double power){
//        frontRight.setPower(0.9*power);
//        frontLeft.setPower(1.4*power);
//        backRight.setPower(1.3*power);
//        backLeft.setPower(0.8*power);
        frontRight.setPower(power);
        frontLeft.setPower(power);
        backRight.setPower(power);
        backLeft.setPower(power);
    }

    public void stopMoving(){
        frontLeft.setPower(0.0d);
        frontRight.setPower(0.0d);
        backLeft.setPower(0.0d);
        backRight.setPower(0.0d);
    }
}
