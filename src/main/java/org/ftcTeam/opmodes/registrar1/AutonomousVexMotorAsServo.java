package org.ftcTeam.opmodes.registrar1;


import android.util.Base64;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

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
import org.ftcbootstrap.components.TimerComponent;
import org.ftcbootstrap.components.operations.motors.GamePadTankDrive;

import java.io.UnsupportedEncodingException;
import java.sql.Time;

/**
 * Note:  It is assumed that the proper registry is used for this set of demos. To confirm please
 * search for "Enter your custom registry here"  in  {@link org.ftcTeam.FTCTeamControllerActivity}
 * <p/>
 * Summary:  Use an Operation class to perform a tank drive using the gamepad joysticks.
 * See: {@link GamePadTankDrive}
 */

@Autonomous
public class AutonomousVexMotorAsServo extends ActiveOpMode {

    CRServo vexMotor;
    TimerComponent timer;
    boolean goForward;



    /**
     * Implement this method to define the code to run when the Init button is pressed on the Driver station.
     */
    @Override
    protected void onInit() {
        getTelemetryUtil().addData("Init", getClass().getSimpleName() + " onInit.");
        getTelemetryUtil().sendTelemetry();

        vexMotor = hardwareMap.crservo.get("servo1");
        goForward = true;
    }

    @Override
    protected void onStart() throws InterruptedException {
        super.onStart();

        getTelemetryUtil().addData("Start", getClass().getSimpleName() + " onStart.");
        getTelemetryUtil().sendTelemetry();

        timer = getTimer();
    }

    /**
     * Implement this method to define the code to run when the Start button is pressed on the Driver station.
     * This method will be called on each hardware cycle just as the loop() method is called for event based Opmodes
     *
     * @throws InterruptedException
     */
    @Override
    protected void activeLoop() throws InterruptedException {
        getTelemetryUtil().addData("activeLoop", "stuff");

        if (goForward) {
            vexMotor.setPower(1.0d);
        }
        else {
            vexMotor.setPower(-1.0d);
        }

        if (timer.targetReached(3.0d)) {
            goForward = false;
        }

        getTelemetryUtil().sendTelemetry();
    }

    private void autoPlay() {

    }

}
