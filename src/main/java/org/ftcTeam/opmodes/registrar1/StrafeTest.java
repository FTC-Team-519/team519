package org.ftcTeam.opmodes.registrar1;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.ftcbootstrap.ActiveOpMode;
import org.ftcbootstrap.components.TimerComponent;

@TeleOp
public class StrafeTest extends ActiveOpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private int sillyCounter = 0;

    private double[] motorPowers = new double[4];
    private static final int FRONT_LEFT  = 0;
    private static final int FRONT_RIGHT = 1;
    private static final int BACK_LEFT   = 2;
    private static final int BACK_RIGHT  = 3;

    private static final float DEAD_ZONE = 0.2f;
    private static final double MAX_SPEED = 1.0d;

    boolean UpBeingPressed;
    boolean DownBeingPressed;

    TimerComponent timerComponent;

    int previousTickCount = 0;


    double[] corrections = new double[4];
    double speedRatio = 1.0d;

    int currMotorAdjust;

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

        speedRatio = 1.0d;

        for (int i = 0; i < 4; i++)
            corrections[i] = 1.0;

        currMotorAdjust = 0;
        UpBeingPressed = false;
        DownBeingPressed = false;
    }

    @Override
    protected void onStart() throws InterruptedException {
        super.onStart();

        getTelemetryUtil().addData("Start", getClass().getSimpleName() + " onStart.");
        getTelemetryUtil().sendTelemetry();

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

        if (gamepad1.x) currMotorAdjust = 0;
        if (gamepad1.y) currMotorAdjust = 1;
        if (gamepad1.a) currMotorAdjust = 2;
        if (gamepad1.b) currMotorAdjust = 3;

        if (gamepad1.right_bumper) {
            speedRatio += 0.001d;
        }

        if (gamepad1.left_bumper) {
            speedRatio -= 0.001d;
        }

        if (gamepad1.dpad_up && !UpBeingPressed) {
            corrections[currMotorAdjust] += 0.025;
            UpBeingPressed = true;
        }
        else if (UpBeingPressed && !gamepad1.dpad_up) {
            UpBeingPressed = false;
        }

        if (gamepad1.dpad_down && !DownBeingPressed) {
            corrections[currMotorAdjust] -= 0.025;
            DownBeingPressed = true;
        }
        else if (DownBeingPressed && !gamepad1.dpad_down) {
            DownBeingPressed = false;
        }

        if(gamepad1.dpad_left) {
            strafeLeft(speedRatio, corrections[1], corrections[3], corrections[0], corrections[2]);
        }
        else if (gamepad1.dpad_right) {
            strafeRight(speedRatio, corrections[1], corrections[3], corrections[0], corrections[2]);
        }
        else {
            stopMoving();
        }

        getTelemetryUtil().addData("frontleft: ", corrections[0]);
        getTelemetryUtil().addData("frontright: ", corrections[1]);
        getTelemetryUtil().addData("backleft: ", corrections[2]);
        getTelemetryUtil().addData("backright: ", corrections[3]);
        getTelemetryUtil().addData("speedRatio: ", speedRatio);
        getTelemetryUtil().sendTelemetry();
    }

    public void strafeLeft (double power, double fr, double br, double fl, double bl) {

        frontRight.setPower(fr*power);
        backRight.setPower(br*(-power));
        frontLeft.setPower(fl*(-power));
        backLeft.setPower(bl*power);
    }
    public void strafeRight (double power, double fr, double br, double fl, double bl) {
        frontRight.setPower(fr*(-power));
        backRight.setPower(br*power);
        frontLeft.setPower(fl*power);
        backLeft.setPower(bl*(-power));
    }

    public void stopMoving(){
        frontLeft.setPower(0.0d);
        frontRight.setPower(0.0d);
        backLeft.setPower(0.0d);
        backRight.setPower(0.0d);
    }
}
