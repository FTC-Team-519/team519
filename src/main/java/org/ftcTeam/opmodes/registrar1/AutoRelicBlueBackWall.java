package org.ftcTeam.opmodes.registrar1;

import android.graphics.Bitmap;
import android.util.Base64;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.sun.tools.javac.util.ForwardingDiagnosticFormatter;
import com.vuforia.Image;
import com.vuforia.VuMarkTargetResult;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.ftcbootstrap.ActiveOpMode;

import java.io.UnsupportedEncodingException;

//Actullay red
@Autonomous
public class AutoRelicBlueBackWall extends ActiveOpMode {

    //License key
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

    private static final double JEWEL_BASE_SHOULDER_POSITION = 0.51;
    private static final double JEWEL_FIRST_ELBOW_POSITION = 0.15;
    private static final double JEWEL_SECOND_ELBOW_POSITION = 0.23;
    private static final double SHOULDER_KNOCK_LEFT = 0.4;
    private static final double SHOULDER_KNOCK_RIGHT = 0.6;
    public static final float STARTING_SHOULDER_POSITION = 0.96f;
    public static final float STARTING_ELBOW_POSITION = 0.91f;
    public static final double ELBOW_MOVEMENT_INCREMENT = 0.003;
    public static final double JEWEL_MAXIMUM_POSITION = 0.54;

    //public static final int STARTING_STEP = 0;
    public static final int STARTING_STEP = 0;



    // Assets
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor lift;
    private Servo servoLeft;
    private Servo servoRight; //clampLeft, clampRight, shoulder, elbow
    private Servo clampLeft;
    private Servo clampRight;
    private Servo shoulder;
    private Servo elbow;

    private ColorSensor color;

    private static final float shoulderInc = .005f;
    private static final float elbowInc = .002f;
    // Vuforia init
    private VuforiaTrackables relicTrackables;
    private VuforiaTrackable relicTemplate;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables ftc2017Trackables;
  //  private ByteBuffer imgBuffer;
    private int byte1;
    private int byte2;
    private int byte3;
    private int byte0;


    private final static float CLOSED_GRIPPER = .7f;
    private final static float GRABBED_GRIPPER = .55f;
    private final static float OPEN_GRIPPER = .35f;

    private final static float HIGH_BATTERY = .0f;
    private final static float LOW_BATTERY = .0f;
/*
    private int red = color.red();
    private int green = color.green();
    private int blue = color.blue();
*/
    private double desiredJewelElbowPosition;

    RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;

    private int step = STARTING_STEP; //0
    private static String convert(byte[] thing) {
        try {
            return new String(thing, "US-ASCII");
        }
        catch (UnsupportedEncodingException uee) {
            return "Could not convert!!!";
        }
    }

    public static Image getImageFromFrame(VuforiaLocalizer.CloseableFrame frame, int format) {

        long numImgs = frame.getNumImages();
        for (int i = 0; i < numImgs; i++) {
            if (frame.getImage(i).getFormat() == format) {
                return frame.getImage(i);
            }//if
        }//for

        return null;
    }

    @Override
    protected void onInit() {
        getTelemetryUtil().addData("Init", getClass().getSimpleName() + " onInit."); //Show the init status
        getTelemetryUtil().sendTelemetry(); //Push update

        color = hardwareMap.colorSensor.get("color");

        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        SetDriveDirection(DriveDirection.Forwards);
        /*frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);*/

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clampLeft = hardwareMap.servo.get("clampLeft");
        clampRight = hardwareMap.servo.get("clampRight");
        shoulder = hardwareMap.servo.get("shoulder");
        elbow = hardwareMap.servo.get("elbow");

        lift = hardwareMap.dcMotor.get("lift");
        //lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setPower(0.0);
        shoulder.setPosition(STARTING_SHOULDER_POSITION);
        elbow.setPosition(STARTING_ELBOW_POSITION);

        // NOTE: Will be eventually by the time start is executed
        desiredJewelElbowPosition = STARTING_ELBOW_POSITION;

        //SetGrabber(GrabberState.Open);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()); //camera id
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId); //init parameters
        parameters.vuforiaLicenseKey = KEY; //set license key
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK; //set camera direction (front or back)
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters); //create vuforia object
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark"); //load the sample image to figure out
        relicTrackables.activate(); //activate it
        relicTemplate = relicTrackables.get(0); //fetch the first image
        //vuforia.setFrameQueueCapacity(1);
        //Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

        try {
          //  Thread.sleep(3000);

            //CheckImage(vuforia.getFrameQueue().take().getImage(0));

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /*
    private ArrayList<Double> RGBtoHSV(int r, int g, int b) {
        ArrayList<Double> vals = new ArrayList<Double>();
        double red = r / 255.0;
        double green = g / 255.0;
        double blue = b / 255.0;
        double cmax= Math.max(Math.max(red, green), blue);
        double cmin= Math.min(Math.min(red, green), blue);
        double luminance = (cmax+cmin)/2;
        double hue;
        double saturation;

        if (cmax == cmin) {
            hue = 0;
            saturation = 0;
        } else {
            double delta = cmax-cmin;
            if (luminance < 0.5) {
                saturation = delta / (cmax + cmin);
            } else {
                saturation = delta / (2 - cmax - cmin);
            }
            if (red == cmax) {
                hue = (green - blue) / delta;
            } else if (green == cmax) {
                hue = 2 + (blue - red) / delta;
            } else {
                hue = 4 + (red - green) / delta;
            }
            hue /= 6;
            if (hue < 0) {
                hue += 1;
            } else if (hue > 1) {
                hue -= 1;
            }
        }
        vals.add(hue);
        vals.add(luminance);
        vals.add(saturation);
        return vals;
    }*/

    public void RGBtoHSLEclipse(int r,int g,int b, double[] hsl) {
        double red = r / 255.0;
        double green = g / 255.0;
        double blue = b / 255.0;
        double cmax= Math.max(Math.max(red, green), blue);
        double cmin= Math.min(Math.min(red, green), blue);
        double  luminance = (cmax+cmin)/2;
        double hue = 0;
        double saturation = 0;
        if (cmax == cmin) {
            hue = 0;
            saturation = 0;
        } else {
            double delta = cmax-cmin;
           /* if (luminance < 0.5) {
                saturation = delta / (cmax + cmin);
            } else {
                saturation = delta / (2 - cmax - cmin);
            }*/
            saturation = delta / (1f - Math.abs(2f * luminance - 1f));
            if (red == cmax) {
                hue = (green - blue) / delta;
            } else if (green == cmax) {
                hue = 2 + (blue - red) / delta;
            } else {
                hue = 4 + (red - green) / delta;
            }
            hue /= 6;
            if (hue < 0) {
                hue += 1;
            } else if (hue > 1) {
                hue -= 1;
            }
        }

        hsl[0] = hue;
        hsl[1] = saturation;
        hsl[2] = luminance;

    }
    public void RGBToHSL(int r, int g, int b, float[] hsl) {
        final float rf = r / 255f;
        final float gf = g / 255f;
        final float bf = b / 255f;

        final float max = Math.max(rf, Math.max(gf, bf));
        final float min = Math.min(rf, Math.min(gf, bf));
        final float deltaMaxMin = max - min;

        float h, s;
        float l = (max + min) / 2f;

        if (max == min) {
            // Monochromatic
            h = s = 0f;
        } else {
            if (max == rf) {
                h = ((gf - bf) / deltaMaxMin) % 6f;
            } else if (max == gf) {
                h = ((bf - rf) / deltaMaxMin) + 2f;
            } else {
                h = ((rf - gf) / deltaMaxMin) + 4f;
            }

            s = deltaMaxMin / (1f - Math.abs(2f * l - 1f));
        }

        hsl[0] = (h * 60f) % 360f;
        hsl[1] = s;
        hsl[2] = l;
    }

    private String getColor(double hue, double sat, double lum) {
        String ret = "Unknown";
        if (hue <= 320 || hue <= 3) {
            if (lum >= .20 && lum <= .70) {
                if (sat >= .40) {
                    ret = "Red";
                    return ret;
                }
            }
        }
       /* if (hue <= 5 || hue >= 360) {
            if (lum >= .40) {
                if (sat <= 40) {
                    ret = "Red";
                }
            }
        }
       /* if (hue >= 320 || hue <= 10) {
            if (lum >= 20 && lum <= 70) {
              //  if (sat >= 40) {
                    ret = "Red";
              //  }
            }
        }*/
        if (hue >= 220 && hue <= 245) {
            if (lum >= 20 && lum <= 70) {
               // if (sat >= 40) {
                    ret = "Blue";
                //}
            }
        }
        return ret;
    }

    private void CheckImage(Image image) {
        getTelemetryUtil().addData("TEst", "yeah"); // sending 500 times...
        getTelemetryUtil().sendTelemetry();
        Bitmap bm = Bitmap.createBitmap(image.getWidth(),image.getHeight(),Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(image.getPixels());

       int[] pix = new int[image.getWidth() * image.getHeight()];
        bm.getPixels(pix, 0, image.getWidth(), 0, 0, image.getWidth(), image.getHeight());

        for (int y = 0; y < bm.getHeight(); y++){
            for (int x = 0; x < bm.getWidth(); x++){
               // int index = y * image.getWidth() + x;
               // int red = (pix[index] >> 16) & 0xff;     //bitwise shifting
                //int green = (pix[index] >> 8) & 0xff;
                //int blue = pix[index] & 0xff;
               /* int pix = bm.getPixel(x,y);
                int red = Color.red(pix);
                int green = Color.green(pix);
                int blue = Color.blue(pix);
                float[] hsv = new float[3];*/
                //Color.RGBToHSV(red,green,blue,hsv);
                //Color.colorToHSV(Color.rgb(red,green,blue), hsv);
              //  getTelemetryUtil().addData("Stats", hsv[0] + " | " + hsv[1] + " | " + hsv[2]);
               /* if ((hsv[0] >= 0 && hsv[0] <= 10) || (hsv[0] >= 350)) {// h
                    if (hsv[1] >= .50) {//s
                        if (hsv[2] >= .70) {
                            getTelemetryUtil().addData("Red", "Yes. ");
                        }
                    }
                }*/
               // double[] hsl = new double[3];

                //RGBtoHSLEclipse(Color.red(pix),Color.green(pix),Color.blue(pix),hsl);

                //getTelemetryUtil().addData("Sat Lum", hsl[1] + " | " + hsl[2]);
                /*if (getColor(hsl[0], hsl[1], hsl[2]) != "Unknown") {
                   // getTelemetryUtil().addData("Color" + getColor(hsl[0], hsl[1], hsl[2]), "Found");
                }*/
              //  getTelemetryUtil().addData("PixelDat", "Red " + red + " Green " + green + " Blue " + blue);
                getTelemetryUtil().sendTelemetry();
            }
        } // gets the array of pixels.
        getTelemetryUtil().addData("Done", "Finished search");
        getTelemetryUtil().sendTelemetry();
        // convert to rgb888



    }
    private boolean doThing = true;
    @Override
    protected void activeLoop() throws InterruptedException {

        if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);

            if (vuMark == RelicRecoveryVuMark.LEFT) {
                getTelemetryUtil().addData("Target", "LEFT");
            } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                getTelemetryUtil().addData("Target", "CENTER");
            } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                getTelemetryUtil().addData("Target", "RIGHT");
            } else {
                getTelemetryUtil().addData("Target", "UNSEEN :-(");
            }
        }

        switch (step) {

            case 0://start
                double result = Double.POSITIVE_INFINITY;
                int i = 0;
                for (VoltageSensor sensor : hardwareMap.voltageSensor) {
                    ++i;
                    double voltage = sensor.getVoltage();
                    if (voltage > 0) {
                        result = Math.min(result, voltage);
                    }
                    getTelemetryUtil().addData("Voltage[" + i + "]", result);
                }

                // setGrabber(GrabberState.Closed);
                ++step;
                //step = 9;//testing purposes, skips jewel
                break;
            case 1:
                shoulder.setPosition(JEWEL_BASE_SHOULDER_POSITION);
                ++step;
                break;
            case 2:
                double shoulderPosition = shoulder.getPosition();
                getTelemetryUtil().addData("Shoulder", shoulderPosition);
                if (shoulderPosition >= JEWEL_BASE_SHOULDER_POSITION) {
                    getTelemetryUtil().addData("Shoulder Found", "TRUE");
                    ++step;
                }
                break;
            case 3:
                desiredJewelElbowPosition = elbow.getPosition();
                ++step;
                break;
            case 4:
                desiredJewelElbowPosition -= ELBOW_MOVEMENT_INCREMENT;
                elbow.setPosition(desiredJewelElbowPosition);
                double firstElbowPosition = elbow.getPosition();
                getTelemetryUtil().addData("Elbow", firstElbowPosition);
                if (firstElbowPosition <= JEWEL_FIRST_ELBOW_POSITION) {
                    getTelemetryUtil().addData("Elbow Found", "FIRST");
                    ++step;
                }
                break;
            case 5: //look at color
                int red = color.red();
                int green = color.green();
                int blue = color.blue();

                getTelemetryUtil().addData("red: ", red);
                getTelemetryUtil().addData("green: ", green);
                getTelemetryUtil().addData("blue: ", blue);

                if ((blue > 5) || (blue > red)) { //knock blue jewel (red jewel stays)
                    shoulder.setPosition(SHOULDER_KNOCK_LEFT);
                    step++;
                } else if ((red > 5) || (red > blue)) {
                    shoulder.setPosition(SHOULDER_KNOCK_RIGHT);
                    step++;
                } else if (getTimer().targetReached(0.01)) {
                    shoulderPosition = shoulder.getPosition();
                    if (shoulderPosition >= JEWEL_MAXIMUM_POSITION) {
                        ++step;
                    } else {
                        shoulder.setPosition(shoulderPosition + 0.005);
                    }

                }
                break;
            case 6:
                desiredJewelElbowPosition = elbow.getPosition();
                ++step;
                break;
            case 7:
                desiredJewelElbowPosition += ELBOW_MOVEMENT_INCREMENT;
                elbow.setPosition(desiredJewelElbowPosition);
                double currentElbowPosition = elbow.getPosition();
                getTelemetryUtil().addData("Elbow", currentElbowPosition);
                if (currentElbowPosition >= STARTING_ELBOW_POSITION) {
                    getTelemetryUtil().addData("Elbow Found", "FIRST");
                    ++step;
                }
                break;
            case 8:
                shoulder.setPosition(STARTING_SHOULDER_POSITION);
                if (getTimer().targetReached(0.7)) {
                    step++;
                }
                break;
            case 9: // raising lift to go over glyph
                lift.setPower(.52);
                if (getTimer().targetReached(.75)) {
                    ++step;
                    lift.setPower(0.0);
                }
                break;
            case 10:
                setGrabber(GrabberState.Open);
                if (getTimer().targetReached(1.5)) {
                    step++;
                }
                break;
            case 11:
                lift.setPower(-0.1);
                if (getTimer().targetReached(.25)) { // .43
                    ++step;
                    lift.setPower(0.0);
                }
                break;
            case 12:
                if (getTimer().targetReached(1.0)) {
                    setGrabber(GrabberState.Grabbed);
                    ++step;
                }
                break;
            case 13:
                if (getTimer().targetReached(1.0d)) {
                    ++step;
                }
                break;
            case 14:
                lift.setPower(.4);
                if (getTimer().targetReached(.3)) {
                    ++step; //normal
                    //step = 9999999; //testing purposes, only lift
                    //lift.setPower(0.0);
                    lift.setPower(0.15); // stall motor to keep glyph from coming down
                }
                break;
            case 15: // first move off the plate
                SetDriveDirection(DriveDirection.Forwards);
                forward(0.2);
                ++step;
                break;
            case 16:
                if (getTimer().targetReached(getForwardDuration())) { // how much to move off the plate
                    stopMoving();
                    ++step;
                }
                break;
            case 17: // move to column
                if (vuMark == RelicRecoveryVuMark.CENTER || vuMark == RelicRecoveryVuMark.RIGHT) {
                    strafeRightSlow();
                } else {
                    strafeLeftSlow();
                }
                ++step;
                break;
            case 18:
                if (getTimer().targetReached(getSecondStrafeDuration(vuMark))) {
                    stopMoving();
                    ++step;
                }
                break;
            case 19:
                turnRight(0.38, false);
                ++step;
                break;
            case 20:
                if (getTimer().targetReached(getSecondTurnDuration(vuMark))) {
                    stopMoving();
                    step++;
                }
                break;
            case 21:
                // move forward after strafe and turn
                forward(.15);
                step = 40;
                break;
            case 40:
                if (getTimer().targetReached(getSecondForwardDuration(vuMark))) {
                    stopMoving();
                    step = 22;
                }
                break;
            case 22:
                //setGrabber(GrabberState.Open);
                clampRight.setPosition(OPEN_GRIPPER);
                clampLeft.setPosition(1 - CLOSED_GRIPPER); //supposed to turn block into crypto-box
                ++step;
                break;
            case 23:
                if (getTimer().targetReached(1.5)) {
                    // Gripper should be opened fully at this point
                    setGrabber(GrabberState.Open);
                    ++step;
                }
                break;
            case 24://back away
                forward(-0.15);
                ++step;
                break;
            case 25:
                if (getTimer().targetReached(getBackwardDuration(vuMark))) {
                    stopMoving();
                    ++step;
                }
                break;
            case 26:
                lift.setPower(0.0);
            default:
                break;
        }
        getTelemetryUtil().addData("Step: ", "" + step);
        getTelemetryUtil().sendTelemetry();
    }

    private double getTurnDuration(RelicRecoveryVuMark bonusColumn) {
        double turnDuration = 0.30;

        if (bonusColumn == RelicRecoveryVuMark.CENTER) {
            turnDuration = .45;
            getTelemetryUtil().addData("Turn: ", "CENTER");
        }
        else if (bonusColumn == RelicRecoveryVuMark.RIGHT) {
            getTelemetryUtil().addData("Turn: ", "RIGHT");
            turnDuration = .635;
        }
        else {
            getTelemetryUtil().addData("Turn: ", "LEFT");
        }

        return turnDuration;
    }

    private double getSecondForwardDuration(RelicRecoveryVuMark bonusColumn) {
        double forwardDuration = .3;
        return forwardDuration;
    }
    private double getForwardDuration() { // how far we should move off the plate
        double forwardDuration = 1.2;
       /* if (testVoltage() > 12.8) {
            forwardDuration = 1.20;
            getTelemetryUtil().addData("Yes", "> than 13");
            getTelemetryUtil().sendTelemetry();
        }*/
/*
        if (bonusColumn == RelicRecoveryVuMark.CENTER) {
            forwardDuration = 1.2;
            getTelemetryUtil().addData("Forward: ", "CENTER");
        }
        else if (bonusColumn == RelicRecoveryVuMark.RIGHT) {
            forwardDuration = 1.2;
            getTelemetryUtil().addData("Forward: ", "RIGHT");
        }
        else {
            getTelemetryUtil().addData("Forward: ", "LEFT");
        }
*/
        return forwardDuration;
    }

    private double getStrafeDuration(RelicRecoveryVuMark bonusColumn) {
        double strafeDuration = 0.00;

        if (bonusColumn == RelicRecoveryVuMark.CENTER) {
            strafeDuration = 0.25;
            getTelemetryUtil().addData("Strafe: ", "CENTER");
        }
        else if (bonusColumn == RelicRecoveryVuMark.RIGHT) {
            strafeDuration = 0.7;
            getTelemetryUtil().addData("Strafe: ", "RIGHT");
        }
        else {
            getTelemetryUtil().addData("Strafe: ", "LEFT");
        }

        return strafeDuration;
    }
    private double getSecondStrafeDuration(RelicRecoveryVuMark bonusColumn) { // how much to strafe to reach the column
        double strafeDuration = 0.250;

        if (bonusColumn == RelicRecoveryVuMark.CENTER) {
            strafeDuration = 1.050;
            getTelemetryUtil().addData("Strafe: ", "CENTER");
        }
        else if (bonusColumn == RelicRecoveryVuMark.RIGHT) {
            strafeDuration = 2.25;
            getTelemetryUtil().addData("Strafe: ", "RIGHT");
        }
        else {
            getTelemetryUtil().addData("Strafe: ", "LEFT");
        }

        return strafeDuration;
    }
    private double getSecondTurnDuration(RelicRecoveryVuMark bonusColumn) {
        double turnDuration = 0.7;
/*
        if (bonusColumn == RelicRecoveryVuMark.CENTER) {
            strafeDuration = 1.0;
            getTelemetryUtil().addData("Strafe: ", "CENTER");
        }
        else if (bonusColumn == RelicRecoveryVuMark.RIGHT) {
            strafeDuration = 1.0;
            getTelemetryUtil().addData("Strafe: ", "RIGHT");
        }
        else {
            getTelemetryUtil().addData("Strafe: ", "LEFT");
            strafeDuration = 1.0;
        }
*/
        return turnDuration;
    }

    private double getBackwardDuration(RelicRecoveryVuMark bonusColumn) {
        //return getForwardDuration(bonusColumn);
        return 0.5;
        //return 0.40;

    }

    public enum DriveDirection {
        Forwards, Backwards
    }

    public enum GrabberState {
        Grabbed, Open, Closed
    }
    public void setGrabber(GrabberState state) {
        if (state == GrabberState.Open)
        {
            clampLeft.setPosition(1 - OPEN_GRIPPER);
            clampRight.setPosition(OPEN_GRIPPER);
        }
        else if (state == GrabberState.Grabbed)
        {
            clampLeft.setPosition(1 - GRABBED_GRIPPER);
            clampRight.setPosition(GRABBED_GRIPPER);
        }
        else
        {
            clampLeft.setPosition(1 - CLOSED_GRIPPER);
            clampRight.setPosition(CLOSED_GRIPPER);
        }
    }
    public void SetDriveDirection(DriveDirection direction) {
        if (direction == DriveDirection.Forwards) {
            // default
            frontRight.setDirection(DcMotor.Direction.REVERSE);
            backRight.setDirection(DcMotor.Direction.REVERSE);
            frontLeft.setDirection(DcMotor.Direction.FORWARD);
            backLeft.setDirection(DcMotor.Direction.FORWARD);
        } else if (direction == DriveDirection.Backwards) {
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            backRight.setDirection(DcMotor.Direction.FORWARD);
            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotor.Direction.REVERSE);
        }

    }

    public void reverse(double power) {
        forward(-power);
    }

    public void forward(double power) {
        frontRight.setPower(power);
        frontLeft.setPower(power);
        backRight.setPower(power);
        backLeft.setPower(power);
    }

    public void turnLeft(double power, boolean turnOnSpot) {
        frontRight.setPower(power);
        backRight.setPower(power);
        if (turnOnSpot) {
            backLeft.setPower(-power);
            frontLeft.setPower(-power);
        }
    }

    public void turnRight(double power, boolean turnOnSpot) {
        frontLeft.setPower(power);
        backLeft.setPower(power);
        if (turnOnSpot) {
            backRight.setPower(-power);
            frontRight.setPower(-power);
        }
    }

    public void strafeLeft(double power) {

        frontRight.setPower(0.8 * power);
        backRight.setPower(0.9 * -power);
        frontLeft.setPower(1.0 * -power);
        backLeft.setPower(0.725 * power);
    }

    public void strafeRight(double power) {

//        frontRight.setPower(0.65*(-power));
//        backRight.setPower(0.8*power);
//        frontLeft.setPower(1.0*power);
//        backLeft.setPower(0.75*(-power));
        frontRight.setPower(1.0 * (-power));
        backRight.setPower(1.0 * power);
        frontLeft.setPower(1.0 * power);
        backLeft.setPower(1.0 * (-power));
    }

    public void strafeLeftSlow() {
        double pow = .6;
        //double pow = .65;
        //double pow = .9;

        frontRight.setPower(0.5 * pow);
        backRight.setPower(-0.5 * pow);
        frontLeft.setPower(-0.5 * pow);
        backLeft.setPower(0.5 * pow);
    }

    public void strafeRightSlow() {
        double pow = .6;
        //double pow = .55;
        //double pow = .9;

        frontRight.setPower(-0.5 * pow);
        backRight.setPower(0.5 * pow);
        frontLeft.setPower(0.5 * pow);
        backLeft.setPower(-0.5 * pow);
    }

    public void stopMoving() {
        frontLeft.setPower(0.0d);
        frontRight.setPower(0.0d);
        backLeft.setPower(0.0d);
        backRight.setPower(0.0d);
    }
    public double testVoltage() {
        double result = Double.POSITIVE_INFINITY;
        int i = 0;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            ++i;
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        getTelemetryUtil().addData("Voltage tested : ", result);
        getTelemetryUtil().sendTelemetry();
        return result;
    }

    }

