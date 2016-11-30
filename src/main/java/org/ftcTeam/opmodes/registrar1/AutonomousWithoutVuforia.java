package org.ftcTeam.opmodes.registrar1;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.ftcbootstrap.ActiveOpMode;
import org.ftcbootstrap.components.TimerComponent;


@Autonomous
public class AutonomousWithoutVuforia extends ActiveOpMode {

    TimerComponent timer;
    boolean goForward;

    private int step;
    private DcMotor shooter;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor midCollector;
    private DcMotor frontCollector;
    private ColorSensor color;

    /**
     * Implement this method to define the code to run when the Init button is pressed on the Driver station.
     */
    @Override
    protected void onInit() {
        getTelemetryUtil().addData("Init", getClass().getSimpleName() + " onInit.");
        getTelemetryUtil().sendTelemetry();

        frontLeft = hardwareMap.dcMotor.get("motor4");
        frontRight = hardwareMap.dcMotor.get("motor2");
        backLeft = hardwareMap.dcMotor.get("motor3");
        backRight = hardwareMap.dcMotor.get("motor1");
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        midCollector = hardwareMap.dcMotor.get("feeder");
        frontCollector = hardwareMap.dcMotor.get("collector");
        shooter = hardwareMap.dcMotor.get("shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.getController().setMotorZeroPowerBehavior(1, DcMotor.ZeroPowerBehavior.FLOAT);

        color = hardwareMap.colorSensor.get("color");


    }

    @Override
    protected void onStart() throws InterruptedException {
        super.onStart();

        step = 0;

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
        switch(step) {
            case 0:
                shooter.setPower(1.0d);
                if (getTimer().targetReached(2.0d)) {
                    ++step;
                }
                break;
            case 1:
                midCollector.setPower(0.5d);
                if (getTimer().targetReached(2.0d)) {
                    midCollector.setPower(0.0d);
                    shooter.setPower(0.0d);
                    ++step;
                }
                break;
            case 2:
                turnLeft(0.5d, false);
                if (getTimer().targetReached(1.2d)) {
                    stopMoving();
                    ++step;
                }
                break;
            case 3:
                forward(0.5d);
                if (getTimer().targetReached(1.5d)) {
                    stopMoving();
                    ++step;
                }
                break;
            case 4:
                turnRight(0.5d, false);
                if (getTimer().targetReached(1.2d)) {
                    stopMoving();
                    ++step;
                }
                break;
            case 5:
                forward(0.15d);
                int green1 = color.green();
                sleep(300);
                int green2 = color.green();
                if (green2 > green1) {
                    stopMoving();
                    ++step;
                }
            default:
                break;
                // should never hit here
        }
        getTelemetryUtil().sendTelemetry();
    }

    public void forward (double power){
        frontRight.setPower(power);
        frontLeft.setPower(power);
        backRight.setPower(power);
        backLeft.setPower(power);
    }

    public void turnLeft (double power, boolean turnOnSpot){
        frontRight.setPower(power);
        backRight.setPower(power);
        if (turnOnSpot){
            backLeft.setPower(-power);
            frontLeft.setPower(-power);
        }
    }

    public void turnRight (double power, boolean turnOnSpot){
        frontLeft.setPower(power);
        backLeft.setPower(power);
        if (turnOnSpot){
            backRight.setPower(-power);
            frontRight.setPower(-power);
        }
    }

    public void stopMoving(){
        frontLeft.setPower(0.0d);
        frontRight.setPower(0.0d);
        backLeft.setPower(0.0d);
        backRight.setPower(0.0d);
    }
}
