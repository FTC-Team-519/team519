package org.ftcTeam.opmodes.registrar1;


import android.util.Base64;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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

import com.qualcomm.ftcrobotcontroller.R;

import java.io.UnsupportedEncodingException;

/**
 * Autonomous Vuforia reference class that doesn't depend upon any motors, just the camera.
 */

@Autonomous
public class AutonomousVuforia extends ActiveOpMode {
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
     * @return transformed x and y vector values to account for z rotation.
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
    public static final float DESIRED_DEGRESS_BLUE_Z = -90f; // compare against orientation

    private VuforiaLocalizer.Parameters parameters;
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables ftc2016Trackables;

    private VuforiaTrackable wheels;
    private VuforiaTrackable tools;
    private VuforiaTrackable legos;
    private VuforiaTrackable gears;

    private OpenGLMatrix lastKnownLocation = null;

    /**
     * Implement this method to define the code to run when the Init button is pressed on the Driver station.
     */
    @Override
    protected void onInit() {
        parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
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
                        AngleUnit.DEGREES, -90, 90, 0));

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
        getTelemetryUtil().addData("Target", "Looking for target.");
        getTelemetryUtil().addData("Seen: ", ((VuforiaTrackableDefaultListener)gears.getListener()).isVisible() ? "Visible" : "Not Visible");

        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)gears.getListener()).getUpdatedRobotLocation();
        if (robotLocationTransform != null) {
            lastKnownLocation = robotLocationTransform;
        }

        if (lastKnownLocation != null) {
            getTelemetryUtil().addData("Location:", lastKnownLocation.formatAsTransform());

            float[] xyzTranslation = lastKnownLocation.getTranslation().getData();
            float pErrorX = DESIRED_MM_RED_X - xyzTranslation[0];
            float pErrorY = DESIRED_MM_RED_NEAR_Y - xyzTranslation[1];

            Orientation orientation = Orientation.getOrientation(lastKnownLocation,
                    AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            float pErrorDegZ = DESIRED_DEGREES_RED_Z - orientation.thirdAngle;

            // pErrorX for RED side, is an attempt to move forward, but only if facing forward/backward
            // pErrorY for RED side, is an attempt to move sideways, but only if facing forward/backward
            // pErrorDegZ for RED side, is an attempt to rotate to correct orientation
            float xVector = 0.0f;
            if (pErrorX < -20f) {
                xVector = -1.0f;  // FIXME: Should be some proportional value
            } else if (pErrorX > 20f) {
                xVector = 1.0f;
            }

            float yVector = 0.0f;
            if (pErrorY < -20f) {
                yVector = -1.0f;
            } else if (pErrorY > 20f) {
                yVector = 1.0f;
            }

            double[] altered = rotateVector(xVector, yVector, orientation.thirdAngle);

            float zVector = 0.0f;
            if (pErrorDegZ < -2f) {
                zVector = -0.5f;
            } else if (pErrorDegZ > 2f) {
                zVector = 0.5f;
            }
            // FIXME: normalize combination of values applied to all 4 motors
        }
        else {
            getTelemetryUtil().addData("Location:", "unknown");
        }

        getTelemetryUtil().sendTelemetry();
    }

}
