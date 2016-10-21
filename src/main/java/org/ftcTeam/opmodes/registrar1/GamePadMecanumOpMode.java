package org.ftcTeam.opmodes.registrar1;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.ftcTeam.configurations.FTCTeamRobot;
import org.ftcbootstrap.ActiveOpMode;
import org.ftcbootstrap.components.operations.motors.GamePadTankDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import android.os.Handler;
import android.os.SystemClock;
import android.util.Log;
import android.view.ViewParent;


/**
 * Note:  It is assumed that the proper registry is used for this set of demos. To confirm please
 * search for "Enter your custom registry here"  in  {@link org.ftcTeam.FTCTeamControllerActivity}
 * <p/>
 * Summary:  Use an Operation class to perform a tank drive using the gamepad joysticks.
 * See: {@link GamePadTankDrive}
 */

@TeleOp
public class GamePadMecanumOpMode extends ActiveOpMode {
    DcMotor frontleft, frontright, backleft, backright, collectorOne, collectorTwo, liftleft, liftright; //TODO convert to motormap @jake
    Servo flag, clamp, dump;
    private float x, y, z, w, pwr;
    public static double deadzone = 0.2;

    private FTCTeamRobot robot;
    private GamePadTankDrive gamePadTankDrive1, gamePadTankDrive2;
    //servo minmax
    final static double CLAMP_MIN  = 172.0/255.0;
    final static double CLAMP_MID = 220.0/255.0;
    final static double CLAMP_MAX  = 232.0/255.0;
    final static double DUMP_MIN  = 80.0/255.0;
    final static double DUMP_MAX  = 180.0/255.0;

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
        frontleft = hardwareMap.dcMotor.get("motor4");
        frontright = hardwareMap.dcMotor.get("motor2");
        backleft = hardwareMap.dcMotor.get("motor3");
        backright = hardwareMap.dcMotor.get("motor1");
        frontright.setDirection(DcMotor.Direction.REVERSE);
        backright.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    protected void onStart() throws InterruptedException {
        super.onStart();

        //create the operation  to perform a tank drive using the gamepad joysticks.
        //gamePadTankDrive1 = new GamePadTankDrive(this, gamepad1, robot.motor1, robot.motor2);
        //gamePadTankDrive2 = new GamePadTankDrive(this, gamepad1, robot.motor3, robot.motor4);
    }

    /**
     * Implement this method to define the code to run when the Start button is pressed on the Driver station.
     * This method will be called on each hardware cycle just as the loop() method is called for event based Opmodes
     *
     * @throws InterruptedException
     */
    @Override
    protected void activeLoop() throws InterruptedException {
/*
        //update the motors with the gamepad joystick values
        gamePadTankDrive1.update();
        gamePadTankDrive2.update();

        //robot.motor1.setPower(.5);

        getTelemetryUtil().addData("activeLoop", getClass().getSimpleName() + " allcowseatgrass.");
        getTelemetryUtil().sendTelemetry();



        //send any telemetry that may have been added in the above operations
        //getTelemetryUtil().sendTelemetry();
*/
        getJoyVals();
        //updates joyvalues with deadzones, xyzw

        pwr = y; //this can be tweaked for exponential power increase
/*
fr = m1
br = 2
fl = 3
bl = 4
 */
        // Might want to have a more effective combination
        frontright.setPower(Range.clip(-pwr - x-z, -0.5, 0.5));
        frontleft.setPower(Range.clip(-pwr + x+z, -0.5, 0.5));
        backright.setPower(Range.clip(-pwr + x-z, -0.5, 0.5));
        backleft.setPower(Range.clip(-pwr - x+z, -0.5, 0.5));

    }
    public void getJoyVals()
    {
        y = gamepad1.left_stick_y;
        x = gamepad1.left_stick_x;
        z = gamepad1.right_stick_x;
        w = gamepad1.right_stick_y;
        //updates joystick values

        //y *= (y < 0) ? -y : y;

        y = shape(gamepad1.left_stick_y);
        x = shape(gamepad1.left_stick_x);
        z = shape(gamepad1.right_stick_x);
        w = shape(gamepad1.right_stick_y);

        if(Math.abs(x)<deadzone) x = 0;
        if(Math.abs(y)<deadzone) y = 0;
        if(Math.abs(z)<deadzone) z = 0;
        if(Math.abs(w)<0.9) w = 0;
        //checks deadzones
    }
//attempts to make driving easier by making it less sensitive in the begining
// while becoming increasingly senseitve, the farther the joystick is pushed
    //originally written by Luke Adsit, 10/20/16
public float shape(double conInput){
    //return 1.0f;
        //above code is if you want to run into whatever is
        //directly in front of the robot at full speed!
    float shape[] = {0f, 0.2f, 0.25f, 0.3f, 0.375f, 0.45f, 0.525f, 0.65f, 0.75f, 0.875f, 1.0f};
    float conOutput = 0.0f;
    //determines the sign
    boolean neg = false;
    if (conInput < 0) {neg = true;}
            //finds the input and assigns a new value from the double array shape
            for(double i = 0.0; i < 1.0; i += 0.1)
    {
        if ( conInput < i )
        {
            conOutput = shape[(int)((i - 0.1) * 10.0)];
            if(neg){conOutput = -1 * conOutput;}//preserves the sign/direction of input
            return conOutput;
        }
    }
    return 0.0f;
    //"fall-back"... doesn't move,
}
}
/*start
double shape[10] = {0.2, 0.25, 0.3, 0.375, 0.45, 0.525, 0.65, 0.75, 0.875, 1.0}
 conInput = [int} conInput;
double conOutput
        conOutput = shape[conInput]
        //finds the input and assigns a new value from the double array shape
for (int i = 0; i < 1; i += 0.1;)
        {
            if ( conInput < i )
            {
                conOutput= shape[(i- 0.1) * 10];
            }
        }
end*/