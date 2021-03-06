package org.ftcTeam.opmodes.registrar1;

//RED

import android.util.Base64;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

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
@Disabled
public class AutonomousRedDistanceSensor extends ActiveOpMode {
    private static final String A = "QWJaajY5di8vLy8vQUFBQUdTWHZyMEc2TDBrTXJ3TUQwT";
    private static final String B = "zdZRWdNc0lmOEI2WFZ1eEV1UTNCci8wV1d0Tk13dHBSMm";
    private static final String C = "5sZHA5cmsxTk1PbHhWc0VBcXIwbmFac1o0dmlvTlZ0R0Q";
    private static final String D = "2K2tMUXVZd0lrSWlKMXlsZGl5Wjd4U055Qkt0Yk0zeVoy";
    private static final String E = "dFBYMFRFZnYrY1o0L3d5TmhXRVoxMU0wdk9GenVsSUFlc";
    private static final String F = "mRwUTBhNzRzeTZHWWc5UzFtSzRGUDJnbFh0b2tuVy94Um";
    private static final String G = "FBaG1zWnkzaXRjZWhYZCtLYVNRS2JBNzZMOGdGcytFY2x";
    private static final String H = "RQW1XQlFwdVRIRHlLcmVMbzEvaHlxUjRzb1BaUXIxemgy";
    private static final String I = "RXg1RWZ0K01TdzBmaUh3NjhtWG1lWDh1L1ZrMk00QWVsR";
    private static final String J = "ERlSE5UdndoMHJ1dnVYczdGVmRaZDBMWm4yMnR4RStWbm";
    private static final String K = "h6cGt5OUFFbEI5eHZVYkFoZTIwN2lWSU5wSndpZDR3d0l";
    private static final String L = "LZ3VFTEdRMTQK";

    private static final String VALUE = A + B + C + D + E + F + G + H + I + J + K + L;

    private static final byte[] MY_VALUE = Base64.decode(VALUE, Base64.NO_WRAP);
    private static final String KEY = convert(MY_VALUE);

    private DecimalFormat df = new DecimalFormat("#.##");
    private boolean calibration_complete = false;

    int step = 2;
    private boolean missedBeacon = false;

    private static String convert(byte[] thing) {
        try {
            return new String(thing, "US-ASCII");
        }
        catch (UnsupportedEncodingException uee) {
            return "Could not convert!!!";
        }
    }

    private static final double PI = 3.14159d;

    /**
     * Degrees to radians is pi times the angle in degrees divided by 180 degrees.
     * @param degrees
     * @return
     */
    public static double degreesToRadians(double degrees) {
        return PI * degrees / 180.0d;
    }

    /**
     * Returns the transformed x and y vector values to account for z rotation.
     *
     * @param x the x coordinate vector value
     * @param y the y coordinate vector value
     * @param degrees the current rotation around the z axis
     * @returntransformed x and y vector values to account for z rotation.
     */
    public static double[] rotateVector(double x, double y, double degrees) {
        double[] retValue = new double[2];

        double radians = degreesToRadians(degrees);
        double cosineA = Math.cos(radians);
        double sineA   = Math.sin(radians);

        // RotationMatrix = +cos?   -sin?
        //                  +sin?   +cos?
        //
        retValue[0] = (x * cosineA) - (y * sineA);
        retValue[1] = (x * sineA) + (y * cosineA);
        return retValue;
    }

    /**
     * Returns power values normalized across motors so that they are all within the
     * expected [-1.0 .. 1.0] range, which is often not true after combining values for mecanum.
     *
     * @param motorPowers the motor values to normalize to a max point.
     */
    private static void normalizeCombinedPowers(double[] motorPowers) {
        double maxAbsPower = 0.0d;

        for (double motorPower : motorPowers) {
            double tmpAbsPower = Math.abs(motorPower);
            if (tmpAbsPower > maxAbsPower) {
                maxAbsPower = tmpAbsPower;
            }
        }

        if (maxAbsPower > 1.0d) {
            for (int i = 0; i < motorPowers.length; ++i) {
                motorPowers[i] = motorPowers[i] / maxAbsPower;
            }
        }
    }

    /**
     * Returns a power that is reduced proportionally to the maximum speed, which would be
     * no reduction if max speed is 1.0 (or basically 100%).
     *
     * @param input power to potentially be reduced.
     * @return a power that is reduced proportionally to the maximum speed, which would be
     * no reduction if max speed is 1.0 (or basically 100%).
     */
    private static double reducePower(double input) {
        // return input;   // If no reduction desired
        return input * MAX_SPEED;
    }

    public static final float MM_PER_INCH = 25.4f;
    public static final float MM_BOT_WIDTH = 18 * MM_PER_INCH;            // ... or whatever is right for your robot
    public static final float MM_FTC_FIELD_WIDTH = (12*12 - 2) * MM_PER_INCH;   // the FTC field is ~11'10" center-to-center of the glass panels
    public static final float MM_TARGET_CENTER_HEIGHT_Z = (1.5f + (8.5f / 2.0f)) * MM_PER_INCH; // half height of 8.5x11 sheet plus 1.5" above floor
    public static final float MM_NEAR_OFFSET = (1 * 12) * MM_PER_INCH; // one foot away from center
    public static final float MM_FAR_OFFSET = (3 * 12) * MM_PER_INCH;  // three feet away from center

    public static final float MM_RED_BEACON_WALL_X = -(MM_FTC_FIELD_WIDTH/2);
    public static final float MM_BLUE_BEACON_WALL_Y = (MM_FTC_FIELD_WIDTH/2);

    public static final float MM_RED_NEAR_TARGET_Y  = -MM_NEAR_OFFSET;
    public static final float MM_RED_FAR_TARGET_Y   = MM_FAR_OFFSET;
    public static final float MM_BLUE_NEAR_TARGET_X = MM_NEAR_OFFSET;
    public static final float MM_BLUE_FAR_TARGET_X  = -MM_NEAR_OFFSET;

    // FIXME: This should be set to whatever distance is desired back from wall to look at
    //        colors/maybe shoot/and start going straight into buttons once lined up with target
    public static final float DESIRED_MM_DISTANCE_FROM_WALL = 16 * MM_PER_INCH;

    public static final float DESIRED_MM_RED_X      = MM_RED_BEACON_WALL_X + DESIRED_MM_DISTANCE_FROM_WALL;
    public static final float DESIRED_MM_RED_NEAR_Y = MM_RED_NEAR_TARGET_Y;
    public static final float DESIRED_MM_RED_FAR_Y  = MM_RED_FAR_TARGET_Y;
    public static final float DESIRED_DEGREES_RED_Z = 90f;  // compare against orientation

    public static final float DESIRED_MM_BLUE_Y      = MM_BLUE_BEACON_WALL_Y - DESIRED_MM_DISTANCE_FROM_WALL;
    public static final float DESIRED_MM_BLUE_NEAR_X = MM_BLUE_NEAR_TARGET_X;
    public static final float DESIRED_MM_BLUE_FAR_X  = MM_BLUE_FAR_TARGET_X;
    public static final float DESIRED_DEGREES_BLUE_Z = -90f; // compare against orientation

    private VuforiaLocalizer.Parameters parameters;
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables ftc2016Trackables;

    private VuforiaTrackable wheels;
    private VuforiaTrackable tools;
    private VuforiaTrackable legos;
    private VuforiaTrackable gears;

    private OpenGLMatrix lastKnownLocation = null;

    private DcMotor shooter;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor topCollector;
    private DcMotor midCollector;
    private DcMotor frontCollector;
    private ColorSensor colorSensor;
    private I2cDevice ultrasonicSensor;
    public I2cDeviceSynch ultrasonicReader;

    private double[] motorPowers = new double[4];
    private static final int FRONT_LEFT  = 0;
    private static final int FRONT_RIGHT = 1;
    private static final int BACK_LEFT   = 2;
    private static final int BACK_RIGHT  = 3;

    boolean beaconIsRed = false;

    private static final double MAX_SPEED = 0.3d;

    byte[] ultrasonicCache;
    public static final int ULTRASONIC_REG_START = 0x04; //Register to start reading
    public static final int ULTRASONIC_READ_LENGTH = 2; //Number of byte to read



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

        topCollector = hardwareMap.dcMotor.get("topCollector");
        midCollector = hardwareMap.dcMotor.get("feeder");
        shooter = hardwareMap.dcMotor.get("shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.getController().setMotorZeroPowerBehavior(1, DcMotor.ZeroPowerBehavior.FLOAT);

        I2cAddr ultrasonicAddress = new I2cAddr(0x14);
        ultrasonicSensor = hardwareMap.i2cDevice.get("range");
        ultrasonicReader = new I2cDeviceSynchImpl(ultrasonicSensor, ultrasonicAddress, false);
        ultrasonicReader.engage();

        colorSensor = hardwareMap.colorSensor.get("colorSensor1");
        colorSensor.enableLed(false);

        parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = KEY;
        //parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        ftc2016Trackables = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");

        wheels = ftc2016Trackables.get(0);
        tools  = ftc2016Trackables.get(1);
        legos  = ftc2016Trackables.get(2);
        gears  = ftc2016Trackables.get(3);

        wheels.setName("wheels"); // blue-near
        tools.setName("tools"); // red-far
        legos.setName("legos"); // blue-far
        gears.setName("gears"); // red-near

        OpenGLMatrix redNearTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                // This also translates +z to be correct height above floor
                // This translates -y to move along wall toward Red team
                .translation(MM_RED_BEACON_WALL_X, MM_RED_NEAR_TARGET_Y, MM_TARGET_CENTER_HEIGHT_Z)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        gears.setLocation(redNearTargetLocationOnField);

        OpenGLMatrix redFarTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                // This also translates +z to be correct height above floor
                // This translates +y to move along wall away from Red team
                .translation(MM_RED_BEACON_WALL_X, MM_RED_FAR_TARGET_Y, MM_TARGET_CENTER_HEIGHT_Z)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        tools.setLocation(redFarTargetLocationOnField);

        OpenGLMatrix blueNearTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                // This also translates +z to be correct height above floor
                // This translates +x to move along wall toward Blue team
                .translation(MM_BLUE_NEAR_TARGET_X, MM_BLUE_BEACON_WALL_Y, MM_TARGET_CENTER_HEIGHT_Z)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        wheels.setLocation(blueNearTargetLocationOnField);

        OpenGLMatrix blueFarTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                // This also translates +z to be correct height above floor
                // This translates -x to move along wall away from Blue team
                .translation(MM_BLUE_FAR_TARGET_X, MM_BLUE_BEACON_WALL_Y, MM_TARGET_CENTER_HEIGHT_Z)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        legos.setLocation(blueFarTargetLocationOnField);

        // NOTE: +Y translation puts it on front of robot, and -Y to put on back
        // FIXME: Still need +z translation to put at proper height (probably not centered)
        OpenGLMatrix frontCenteredLocationOnRobot = OpenGLMatrix
                .translation(0, MM_BOT_WIDTH/2, MM_BOT_WIDTH/2)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -90, -90, 0));

        OpenGLMatrix backCenteredLocationOnRobot = OpenGLMatrix
                .translation(0, -MM_BOT_WIDTH/2, MM_BOT_WIDTH/2)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -90, -90, 0));

        OpenGLMatrix phoneLocationOnRobot = frontCenteredLocationOnRobot;

        ((VuforiaTrackableDefaultListener)wheels.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)tools.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)legos.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)gears.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

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
        ftc2016Trackables.activate();

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
        ultrasonicCache = ultrasonicReader.read(ULTRASONIC_REG_START, ULTRASONIC_READ_LENGTH);

        switch(step) {
            case 0://spin up wheel
                shooter.setPower(1.0d);
                if (getTimer().targetReached(1.8d)) {
                    ++step;
                }
                break;
            case 1://shoot
                topCollector.setPower(0.5d);
                midCollector.setPower(0.5d);
                if (getTimer().targetReached(0.6d)) {
                    topCollector.setPower(0.0d);
                    midCollector.setPower(0.0d);
                    shooter.setPower(0.0d);
                    ++step;
                    //step = 99999;
                }
                break;
            case 2://move forward then stop
                forward(0.5d);
                //if (getTimer().targetReached(1.2d)) {
                if (getTimer().targetReached(0.9d)) {
                    stopMoving();
                    ++step;
                }
                break;
            case 3: //turn towards first beacon
                turnRight(0.5d, true);
                //if (getTimer().targetReached(1.2d)) {
                if (getTimer().targetReached(0.85d)) {
                    stopMoving();
                    ++step;
                }
                break;

            case 4:
                forward(-0.5d);
                //if (getTimer().targetReached(1.0d)) {
                if (getTimer().targetReached(0.9d)) {
                    stopMoving();
                    ++step;
                }
                break;
            case 5:
                isVisible = ((VuforiaTrackableDefaultListener)gears.getListener()).isVisible();
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)gears.getListener()).getUpdatedRobotLocation();
                getTelemetryUtil().addData("Target", "Looking for target.");
                if (robotLocationTransform != null) {
                    lastKnownLocation = robotLocationTransform;
                }

                if (lastKnownLocation != null && isVisible) {
                    Orientation orientation = Orientation.getOrientation(lastKnownLocation,
                            AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    float pErrorDegZ = DESIRED_DEGREES_RED_Z - orientation.thirdAngle;

                    double zVector = 0.0f;
                    if (pErrorDegZ < -17f) {
                        zVector = -0.15f;
                    } else if (pErrorDegZ > 17f) {
                        zVector = 0.15f;
                    }

                    turnLeft(zVector, true);

                    if (Math.abs(pErrorDegZ) < 17f) {
                        stopMoving();

                        ++step;
                    }
                }
                else {
                    //turnLeft(0.12f, true);
                    turnLeft(0.17f, true);
                }

                break;
            case 6:
                isVisible = ((VuforiaTrackableDefaultListener)gears.getListener()).isVisible();
                robotLocationTransform = ((VuforiaTrackableDefaultListener)gears.getListener()).getUpdatedRobotLocation();
                getTelemetryUtil().addData("Target", "Looking for target.");
                if (robotLocationTransform != null) {
                    lastKnownLocation = robotLocationTransform;
                }

                if (lastKnownLocation != null && isVisible) {
                    getTelemetryUtil().addData("Location:", lastKnownLocation.formatAsTransform());
                    float[] xyzTranslation = lastKnownLocation.getTranslation().getData();
                    float pErrorY = DESIRED_MM_RED_NEAR_Y - xyzTranslation[1];

                    double yVector = 0.0f;
                    if (pErrorY < -20f) {
                        //yVector = -0.15f;
                        yVector = 0.4f;
                    } else if (pErrorY > 20f) {
                        //yVector = 0.15f;
                        yVector = -0.4f;
                    }

                    strafeRight(yVector);

                    if (Math.abs(pErrorY) < 6f) {
                        stopMoving();

                        ++step;
                    }
                }
                else {
                    getTelemetryUtil().addData("Location:", "unknown");
                    stopMoving();
                }
                break;
            case 7://move toward first beacon
                forward(-0.13d);
                //if (getTimer().targetReached(1.5d)) {
                //if (getTimer().targetReached(0.75d)) {
                if (ultrasonicCache[0] < 7 || ultrasonicCache[0] > 150) {//back up until it is ten centimeters away from wall
                    stopMoving();
                    ++step;
                }
                break;
            case 8://move away from beacon
                forward(-0.2d);
                //if (getTimer().targetReached(0.3d)) {
                if (getTimer().targetReached(0.3d)) {
                    stopMoving();
                    ++step;
                }
            case 9://move away from beacon
                forward(0.2d);
                //if (getTimer().targetReached(0.3d)) {
                if (getTimer().targetReached(0.3d)) {
                    stopMoving();
                    if (false/*missedBeacon*/)
                        step = 999;//give up and quit if not desired color
                    else
                        ++step;
                }
            case 10:
                if (getTimer().targetReached(1.0)) {
                    beaconIsRed = colorSensor.red() > colorSensor.blue();
                    getTelemetryUtil().addData("red:", " " + colorSensor.red());
                    getTelemetryUtil().addData("blue:", " " + colorSensor.blue());
                    ++step;
                }
                break;
            case 11:
                if(beaconIsRed)
                {
                    ++step;
                    //step = 20;
                }
                else
                {
                    if(getTimer().targetReached(3.7))
                    {
                        step = 7;
                        missedBeacon = true;
                    }
                }
                break;
            case 12:
                forward(0.5d);
                if (getTimer().targetReached(0.30d)) {
                    stopMoving();
                    ++step;
                }
                break;
            case 13:
                strafeLeft(1.0d);
                if (getTimer().targetReached(1.6d)) {
                    stopMoving();
                    ++step;
                }
                break;
            case 14:
                isVisible = ((VuforiaTrackableDefaultListener)tools.getListener()).isVisible();
                robotLocationTransform = ((VuforiaTrackableDefaultListener)tools.getListener()).getUpdatedRobotLocation();
                getTelemetryUtil().addData("Target", "Looking for target.");
                if (robotLocationTransform != null) {
                    lastKnownLocation = robotLocationTransform;
                }

                if (lastKnownLocation != null && isVisible) {
                    getTelemetryUtil().addData("Location:", lastKnownLocation.formatAsTransform());
                    float[] xyzTranslation = lastKnownLocation.getTranslation().getData();
                    float pErrorY = DESIRED_MM_RED_FAR_Y - xyzTranslation[1];

                    double yVector = 0.0f;
                    if (pErrorY < -20f) {
                        //yVector = -0.15f;
                        yVector = 0.4f;
                    } else if (pErrorY > 20f) {
                        //yVector = 0.15f;
                        yVector = -0.4f;
                    }

                    strafeRight(yVector);

                    if (Math.abs(pErrorY) < 6f) {
                        stopMoving();

                        ++step;
                    }
                }
                else {
                    getTelemetryUtil().addData("Location:", "unknown");
                    //stopMoving();
                    strafeLeft(0.4);
                }
                break;
            case 15:
                isVisible = ((VuforiaTrackableDefaultListener)tools.getListener()).isVisible();
                robotLocationTransform = ((VuforiaTrackableDefaultListener)tools.getListener()).getUpdatedRobotLocation();
                getTelemetryUtil().addData("Target", "Looking for target.");
                if (robotLocationTransform != null) {
                    lastKnownLocation = robotLocationTransform;
                }

                if (lastKnownLocation != null && isVisible) {
                    Orientation orientation = Orientation.getOrientation(lastKnownLocation,
                            AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    float pErrorDegZ = DESIRED_DEGREES_RED_Z - orientation.thirdAngle;

                    double zVector = 0.0f;
                    if (pErrorDegZ < -2f) {
                        zVector = -0.17f;
                    } else if (pErrorDegZ > 2f) {
                        zVector = 0.17f;
                    }

                    turnLeft(zVector, true);

                    if (Math.abs(pErrorDegZ) < 2f) {
                        stopMoving();

                        ++step;
                    }
                }
                else {
                    turnLeft(0.15f, true);
                }

                break;
            case 16://move toward first beacon
                forward(-0.13d);
                //if (getTimer().targetReached(1.5d)) {
                //if (getTimer().targetReached(0.75d)) {
                if (ultrasonicCache[0] < 7 || ultrasonicCache[0] > 150) {//back up until it is ten centimeters away from wall
                    stopMoving();
                    ++step;
                }
                break;
            case 17://move away from beacon
                forward(-0.2d);
                //if (getTimer().targetReached(0.3d)) {
                if (getTimer().targetReached(0.3d)) {
                    stopMoving();
                    ++step;
                }
            case 18://move away from beacon
                forward(0.2d);
                //if (getTimer().targetReached(0.3d)) {
                if (getTimer().targetReached(0.3d)) {
                    stopMoving();
                    if (missedBeacon)
                        step = 999;//give up and quit if not desired color
                    else
                        ++step;
                }
            case 19:
                if (getTimer().targetReached(1.0)) {
                    beaconIsRed = colorSensor.red() > colorSensor.blue();
                    getTelemetryUtil().addData("red:", " " + colorSensor.red());
                    getTelemetryUtil().addData("blue:", " " + colorSensor.blue());
                    ++step;
                }
                break;
            case 20:
                if(beaconIsRed)
                {
                    ++step;
                    //step = 20;
                }
                else
                {
                    if(getTimer().targetReached(3.7))
                    {
                        step = 16;
                        missedBeacon = true;
                    }
                }
                break;
            case 21:
                forward(0.5d);
                if (getTimer().targetReached(0.30d)) {
                    stopMoving();
                    ++step;
                    ++step;
                }
                break;
        }
//        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)gears.getListener()).getUpdatedRobotLocation();
//        if (robotLocationTransform != null) {
//            lastKnownLocation = robotLocationTransform;
//        }
//
//        if (lastKnownLocation != null && isVisible) {
//            getTelemetryUtil().addData("Location:", lastKnownLocation.formatAsTransform());
//
//            float[] xyzTranslation = lastKnownLocation.getTranslation().getData();
//            float pErrorX = DESIRED_MM_RED_X - xyzTranslation[0];
//            float pErrorY = DESIRED_MM_RED_NEAR_Y - xyzTranslation[1];
//
//            Orientation orientation = Orientation.getOrientation(lastKnownLocation,
//                    AxesReference.EXTRINSIC,AxesOrder.XYZ, AngleUnit.DEGREES);
//
//            float pErrorDegZ = DESIRED_DEGREES_RED_Z - orientation.thirdAngle;
//
//            getTelemetryUtil().addData("Desired: ", "x:" + DESIRED_MM_RED_X + ", y:" + DESIRED_MM_RED_NEAR_Y + ", z:" + DESIRED_DEGREES_RED_Z);
//            getTelemetryUtil().addData("Error: ", "x: " + pErrorX + ", y:" + pErrorY + ", z:" + pErrorDegZ);
//
//            // pErrorX for RED side, is an attempt to move forward, but only if facing forward/backward
//            // pErrorY for RED side, is an attempt to move sideways, but only if facing forward/backward
//            // pErrorDegZ for RED side, is an attempt to rotate to correct orientation
//            boolean found = false;
//
//            double xVector = 0.0f;
//            if (pErrorX < -20f) {
//                xVector = -1.0f;  // FIXME: Should be some proportional value
//            } else if (pErrorX > 20f) {
//                xVector = 1.0f;
//            } else {
//                found = true;
//            }
//
//            //xVector = 0.0f;
//
//            double yVector = 0.0f;
//            if (pErrorY < -20f) {
//                yVector = -1.0f;
//            } else if (pErrorY > 20f) {
//                yVector = 1.0f;
//            } else {
//                found = true;
//            }
//
//            yVector = 0.0f;
//
//            //double[] altered = rotateVector(xVector, yVector, orientation.thirdAngle);
//            double[] altered = rotateVector(xVector, yVector, 90.0);
//            xVector = altered[0];
//            yVector = altered[1];
//            //yVector = 0.0f;
//
//            double zVector = 0.0f;
//            if (pErrorDegZ < -2f) {
//                zVector = -0.5f;
//            } else if (pErrorDegZ > 2f) {
//                zVector = 0.5f;
//            }
//            // FIXME: normalize combination of values applied to all 4 motors
//
//            zVector = 0.0f;
//
//            if (found) {
//                motorPowers[FRONT_RIGHT] = 0.0d;
//                motorPowers[FRONT_LEFT] = 0.0d;
//                motorPowers[BACK_RIGHT] = 0.0d;
//                motorPowers[BACK_LEFT] = 0.0d;
//            } else {
//                motorPowers[FRONT_RIGHT] = -.10d;
//                motorPowers[FRONT_LEFT] = -.10d;
//                motorPowers[BACK_RIGHT] = -.10d;
//                motorPowers[BACK_LEFT] = -.10d;
//            }
///**
// motorPowers[FRONT_RIGHT] = yVector + xVector + zVector;
// motorPowers[FRONT_LEFT]  = yVector - xVector - zVector;
// motorPowers[BACK_RIGHT]  = yVector - xVector + zVector;
// motorPowers[BACK_LEFT]   = yVector + xVector - zVector;
// normalizeCombinedPowers(motorPowers);
// **/
//            /** frontRight.setPower(reducePower(motorPowers[FRONT_RIGHT]));
//             frontLeft.setPower(reducePower(motorPowers[FRONT_LEFT]));
//             backRight.setPower(reducePower(motorPowers[BACK_RIGHT]));
//             backLeft.setPower(reducePower(motorPowers[BACK_LEFT]));
//             **/
//            frontRight.setPower(motorPowers[FRONT_RIGHT]);
//            frontLeft.setPower(motorPowers[FRONT_LEFT]);
//            backRight.setPower(motorPowers[BACK_RIGHT]);
//            backLeft.setPower(motorPowers[BACK_LEFT]);
//        }
//        else {
//            getTelemetryUtil().addData("Location:", "unknown");
//            frontLeft.setPower(0.0d);
//            frontRight.setPower(0.0d);
//            backLeft.setPower(0.0d);
//            backRight.setPower(0.0d);
//        }
//
        getTelemetryUtil().addData("distance", ultrasonicCache[0]);
        getTelemetryUtil().addData("Seen: ", isVisible ? "Visible" : "Not Visible");
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
    public void strafeLeft (double power) {

        frontRight.setPower(0.925*power);
        backRight.setPower(1.0*-power);
        frontLeft.setPower(1.0*-power);
        backLeft.setPower(0.675*power);
    }
    public void strafeRight (double power) {

        frontRight.setPower(0.775*(-power));
        backRight.setPower(0.95*power);
        frontLeft.setPower(1.0*power);
        backLeft.setPower(0.75*(-power));
    }

    public void stopMoving(){
        frontLeft.setPower(0.0d);
        frontRight.setPower(0.0d);
        backLeft.setPower(0.0d);
        backRight.setPower(0.0d);
    }
}
