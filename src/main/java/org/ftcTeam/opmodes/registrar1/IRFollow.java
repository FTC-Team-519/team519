package org.ftcTeam.opmodes.registrar1;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;

import org.ftcTeam.configurations.MotorAndServoRobot;
import org.ftcbootstrap.ActiveOpMode;
import org.ftcbootstrap.components.operations.motors.GamePadTankDrive;


/**
 * Note:  It is assumed that the proper registry is used for this set of demos. To confirm please
 * search for "Enter your custom registry here"  in  {@link org.ftcTeam.FTCTeamControllerActivity}
 * <p/>
 * Summary:  Use an Operation class to perform a tank drive using the gamepad joysticks.
 * See: {@link GamePadTankDrive}
 */

@TeleOp
public class IRFollow extends ActiveOpMode {
    DcMotor frontleft, frontright, backleft, backright; //TODO convert to motormap @jake
    IrSeekerSensor IRSeeker;
    private MotorAndServoRobot robot;
    //servo minmax

    /**
     * Implement this method to define the code to run when the Init button is pressed on the Driver station.
     */
    @Override
    protected void onInit() {
/*
        robot = FTCTeamRobot.newConfig(hardwareMap, getTelemetryUtil());

        //Note The Telemetry Utility is designed to let you organize all telemetry data before sending it to
        //the Driver station via the sendTelemetry command
        getTelemetryUtil().addData("Init", getClass().getSimpleName() + " initializedX.");
        getTelemetryUtil().sendTelemetry(); */

    }

    @Override
    protected void onStart() throws InterruptedException {
        super.onStart();

        frontleft = hardwareMap.dcMotor.get("motor4");
        frontright = hardwareMap.dcMotor.get("motor2");
        backleft = hardwareMap.dcMotor.get("motor3");
        backright = hardwareMap.dcMotor.get("motor1");
        frontright.setDirection(DcMotor.Direction.REVERSE);
        backright.setDirection(DcMotor.Direction.REVERSE);
        IRSeeker = hardwareMap.irSeekerSensor.get("irseeker");
    }

    /**
     * Implement this method to define the code to run when the Start button is pressed on the Driver station.
     * This method will be called on each hardware cycle just as the loop() method is called for event based Opmodes
     *
     * @throws InterruptedException
     */
    @Override
    protected void activeLoop() throws InterruptedException {
        if (IRSeeker.getStrength()>0.1 /*&& IRSeeker.getStrength()< 5*/){
            double angle = IRSeeker.getAngle();

            if (Math.abs(angle) > 10 && Math.abs(angle) < 60){
                double turnPower = angle/360;
                frontleft.setPower(-turnPower);
                backleft.setPower(-turnPower);
                frontright.setPower(turnPower);
                backright.setPower(turnPower);
            }
            else if (Math.abs(angle) < 10){
                double forwardPower = 0.2;
                frontleft.setPower(forwardPower);
                backleft.setPower(forwardPower);
                frontright.setPower(forwardPower);
                backleft.setPower(forwardPower);
            }
        }
        else{
            frontleft.setPower(0);
            frontright.setPower(0);
            backleft.setPower(0);
            backright.setPower(0);
        }
    }
}