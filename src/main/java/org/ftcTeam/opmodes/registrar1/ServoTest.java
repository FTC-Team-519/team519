package org.ftcTeam.opmodes.registrar1;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;

import org.ftcbootstrap.ActiveOpMode;
import org.ftcbootstrap.components.TimerComponent;

import java.text.DecimalFormat;

public class ServoTest extends ActiveOpMode {

    private double position = 0.0d;
    private Servo servo;

    /**
     * Implement this method to define the code to run when the Init button is pressed on the Driver station.
     */
    @Override
    protected void onInit() {

        //Note The Telemetry Utility is designed to let you organize all telemetry data before sending it to
        //the Driver station via the sendTelemetry command
        getTelemetryUtil().addData("Init", getClass().getSimpleName() + " onInit.");

        servo = hardwareMap.servo.get("servo1");
        getTelemetryUtil().sendTelemetry();
    }

    @Override
    protected void onStart() throws InterruptedException {
        super.onStart();

        getTelemetryUtil().addData("Start", getClass().getSimpleName() + " onStart.");
        getTelemetryUtil().sendTelemetry();
        servo.setPosition(position);
    }

    /**
     * Implement this method to define the code to run when the Start button is pressed on the Driver station.
     * This method will be called on each hardware cycle just as the loop() method is called for event based Opmodes
     *
     * @throws InterruptedException
     */
    @Override
    protected void activeLoop() throws InterruptedException {
        if (gamepad1.a) {
            position -= 0.01d;
        }
        else if (gamepad1.b) {
            position += 0.01d;
        }

        if (position < 0.0d) {
            position = 0.0d;
        }
        else if (position > 1.0) {
            position = 1.0;
        }

        servo.setPosition(position);
        getTelemetryUtil().addData("ServoPos:", "" + position);
        getTelemetryUtil().sendTelemetry();
    }
}
