package org.firstinspires.ftc.teamcode.robot.components.camera;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import org.firstinspires.ftc.teamcode.robot.operations.CameraOperation;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 * Created by silver titans on 9/19/17.
 */

public class WebCam {

    private volatile boolean isInitialized;

    // IMPORTANT:  For Phone WebCam, set 1) the camera source and 2) the orientation, based on how your phone is mounted:
    // 1) WebCam Source.  Valid choices are:  BACK (behind screen) or FRONT (selfie side)
    // 2) Phone Orientation. Choices are: PHONE_IS_PORTRAIT = true (portrait) or PHONE_IS_PORTRAIT = false (landscape)
    //
    // NOTE: If you are running on a CONTROL HUB, with only one USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    //
    public static final float CAMERA_FORWARD_DISPLACEMENT  = 7.25f * Field.MM_PER_INCH;   //WebCam is 7 1/4 Inches in front of robot center
    public static final float CAMERA_VERTICAL_DISPLACEMENT = 4.375f * Field.MM_PER_INCH;   //WebCam is 4 3/8 Inches above ground
    public static final float CAMERA_LEFT_DISPLACEMENT     = 0;     //WebCam is centered left to right

    private VuforiaLocalizer vuforia = null;
    Telemetry telemetry;
    HardwareMap hardwareMap;
    OpenGLMatrix vuforiaCameraFromRobot;
    OpenGLMatrix lastLocation = new OpenGLMatrix();
    VuforiaTrackables targetsSkyStone;
    ArrayList<VuforiaTrackable> allTrackables = new ArrayList<>();

    private Servo cameraServo;

    // Constant for Stone Target
    private static final float stoneZ = (float) (2.00f * Field.MM_PER_INCH);

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * Field.MM_PER_INCH;
    private static final float bridgeY = 23 * Field.MM_PER_INCH;
    private static final float bridgeX = 5.18f * Field.MM_PER_INCH;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72f * Field.MM_PER_INCH;
    private static final float quadField  = 36f * Field.MM_PER_INCH;

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmTargetHeight   = (float) ((5.75) * Field.MM_PER_INCH);          // the height of the center of the target image above the floor

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;
    DcMotor ledControl;

    private SkyStoneDetector skyStoneDetector;

    public void init(HardwareMap hardwareMap, Telemetry telemetry, Alliance.Color allianceColor) {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        ledControl = hardwareMap.get(DcMotor.class, "LED");
        cameraServo = hardwareMap.get(Servo.class, "cameraServo");
        cameraServo.setPosition(0.51);
        this.skyStoneDetector = new SkyStoneDetector(allianceColor);
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        final int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId",
                "id", hardwareMap.appContext.getPackageName());
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        Thread vuforiaInitializationThread = new Thread(new Runnable() {
            @Override
            public void run() {
                synchronized (skyStoneDetector) {

                    //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
                    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

                    parameters.vuforiaLicenseKey = Match.VUFORIA_KEY;
                    parameters.cameraDirection = BACK;
                    parameters.useExtendedTracking = false;
                    /**
                     * We also indicate which camera on the RC we wish to use.
                     */
                    parameters.cameraName = webcamName;

                    //  Instantiate the Vuforia engine
                    vuforia = ClassFactory.getInstance().createVuforia(parameters);
                    vuforia.enableConvertFrameToBitmap();
                    vuforia.setFrameQueueCapacity(1);

                    // Load the data sets for the trackable objects. These particular data
                    // sets are stored in the 'assets' part of our application.
                    targetsSkyStone = vuforia.loadTrackablesFromAsset("Skystone");

                    VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
                    stoneTarget.setName("Stone Target");

                    VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
                    blueRearBridge.setName("BlueV2 Rear Bridge");
                    VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
                    redRearBridge.setName("RedV2 Rear Bridge");
                    VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
                    redFrontBridge.setName("RedV2 Front Bridge");
                    VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
                    blueFrontBridge.setName("BlueV2 Front Bridge");
                    VuforiaTrackable red1 = targetsSkyStone.get(5);
                    red1.setName("RedV2 Perimeter 1");
                    VuforiaTrackable red2 = targetsSkyStone.get(6);
                    red2.setName("RedV2 Perimeter 2");
                    VuforiaTrackable front1 = targetsSkyStone.get(7);
                    front1.setName("Front Perimeter 1");
                    VuforiaTrackable front2 = targetsSkyStone.get(8);
                    front2.setName("Front Perimeter 2");
                    VuforiaTrackable blue1 = targetsSkyStone.get(9);
                    blue1.setName("BlueV2 Perimeter 1");
                    VuforiaTrackable blue2 = targetsSkyStone.get(10);
                    blue2.setName("BlueV2 Perimeter 2");
                    VuforiaTrackable rear1 = targetsSkyStone.get(11);
                    rear1.setName("Rear Perimeter 1");
                    VuforiaTrackable rear2 = targetsSkyStone.get(12);
                    rear2.setName("Rear Perimeter 2");


                    allTrackables.addAll(targetsSkyStone);

                    /**
                     * In order for localization to work, we need to tell the system where each target is on the field, and
                     * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
                     * Transformation matrices are a central, important concept in the math here involved in localization.
                     * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
                     * for detailed information. Commonly, you'll encounter transformation matrices as instances
                     * of the {@link OpenGLMatrix} class.
                     *
                     * If you are standing in the RedV2 Alliance Station looking towards the center of the field,
                     *     - The X axis runs from your left to the right. (positive from the center to the right)
                     *     - The Y axis runs from the RedV2 Alliance Station towards the other side of the field
                     *       where the BlueV2 Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
                     *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
                     *
                     * Before being transformed, each target image is conceptually located at the origin of the field's
                     *  coordinate system (the center of the field), facing up.
                     */

                    // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
                    // Rotated it to to face forward, and raised it to sit on the ground correctly.
                    // This can be used for generic target-centric approach algorithms
                    stoneTarget.setLocation(OpenGLMatrix
                            .translation(0, 0, stoneZ)
                            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));


                    //Set the position of the bridge support targets with relation to origin (center of field)
                    blueFrontBridge.setLocation(OpenGLMatrix
                            .translation(-bridgeX, bridgeY, bridgeZ)
                            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

                    blueRearBridge.setLocation(OpenGLMatrix
                            .translation(-bridgeX, bridgeY, bridgeZ)
                            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

                    redFrontBridge.setLocation(OpenGLMatrix
                            .translation(-bridgeX, -bridgeY, bridgeZ)
                            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

                    redRearBridge.setLocation(OpenGLMatrix
                            .translation(bridgeX, -bridgeY, bridgeZ)
                            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

                    //Set the position of the perimeter targets with relation to origin (center of field)
                    red1.setLocation(OpenGLMatrix
                            .translation(quadField, -halfField, mmTargetHeight)
                            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

                    red2.setLocation(OpenGLMatrix
                            .translation(-quadField, -halfField, mmTargetHeight)
                            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

                    front1.setLocation(OpenGLMatrix
                            .translation(-halfField, -quadField, mmTargetHeight)
                            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

                    front2.setLocation(OpenGLMatrix
                            .translation(-halfField, quadField, mmTargetHeight)
                            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

                    blue1.setLocation(OpenGLMatrix
                            .translation(-quadField, halfField, mmTargetHeight)
                            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

                    blue2.setLocation(OpenGLMatrix
                            .translation(quadField, halfField, mmTargetHeight)
                            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

                    rear1.setLocation(OpenGLMatrix
                            .translation(halfField, quadField, mmTargetHeight)
                            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

                    rear2.setLocation(OpenGLMatrix
                            .translation(halfField, -quadField, mmTargetHeight)
                            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));


                    vuforiaCameraFromRobot = OpenGLMatrix
                            .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, ZYX, DEGREES, -90, 90, 0)).inverted();


                    targetsSkyStone.activate();
                    isInitialized = true;
                }
            }
        });
        vuforiaInitializationThread.start();
    }

    /**
     * Tries to find one of the vuMarks on the field.
     *
     * @return name of the vumark found, null if none could be found
     */
    public String findTarget() {
        OpenGLMatrix vuforiaCameraFromTarget = null;
        VuforiaTrackable visibleTarget = null;
        String foundTarget = null;
        // check all the trackable targets to see which one (if any) is visible.
        for (VuforiaTrackable trackable: allTrackables){
            vuforiaCameraFromTarget = ((VuforiaTrackableDefaultListener)trackable.getListener()).getVuforiaCameraFromTarget();
            if (vuforiaCameraFromTarget != null){
                visibleTarget = trackable;
                break;
            }
        }
        if (vuforiaCameraFromTarget != null) {
            OpenGLMatrix targetFromVuforiaCamera = vuforiaCameraFromTarget.inverted();
            lastLocation = visibleTarget.getFtcFieldFromTarget().multiplied(targetFromVuforiaCamera).multiplied(vuforiaCameraFromRobot);
            VectorF translation = lastLocation.getTranslation();
            //Match.log(String.format("Pos (in), {X, Y, Z} = %.1f, %.1f, %.1f",
                    //translation.get(0) / Field.MM_PER_INCH, translation.get(1) / Field.MM_PER_INCH, translation.get(2) / Field.MM_PER_INCH));
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            //Match.log(String.format("Rot (deg), {Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle));
        }
        telemetry.update();
        return visibleTarget != null ? visibleTarget.getName() : null;
    }

    public float getCurrentX() {
        if (lastLocation != null) {
            VectorF translation = lastLocation.getTranslation();
            return
                    translation.get(0);
        } else {
            return -1;
        }
    }

    public float getCurrentY() {
        if (lastLocation != null) {
            VectorF translation = lastLocation.getTranslation();
            return
                    translation.get(1);
        } else {
            return -1;
        }
    }

    public float getCurrentZ() {
        if (lastLocation != null) {
            VectorF translation = lastLocation.getTranslation();
            return
                    translation.get(2);
        } else {
            return -1;
        }
    }

    public float getCurrentTheta() {
        if (lastLocation != null) {
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            return
                    rotation.thirdAngle;
        } else {
            return -1;
        }
    }

    public String getPosition() {
        if (lastLocation != null) {
            VectorF translation = lastLocation.getTranslation();
            return String.format("{X, Y, Z} = %.2f, %.2f, %.2f",
                    translation.get(0) / Field.MM_PER_INCH,
                    translation.get(1) / Field.MM_PER_INCH,
                    translation.get(2) / Field.MM_PER_INCH);
        } else {
            return "Target not found";
        }
    }

    public void turnLedOn() {
        ledControl.setPower(1);
    }
    public void turnLedOff() {
        ledControl.setPower(0);
    }

    public void handleOperation(CameraOperation operation) {
        if (operation.getCameraOperationType() == CameraOperation.CameraOperationType.FLASH_ON) {
            Match.log("Setting led on");
            ledControl.setPower(1);
        }
        else if (operation.getCameraOperationType() == CameraOperation.CameraOperationType.FLASH_OFF) {
            Match.log("Setting led off");
            ledControl.setPower(0);
        }
        else {
            Match.log("Unknown operation: " + operation.getCameraOperationType());
        }
    }


    /**
     * attempt to find sky stone location
     *
     * @return The position of the sky stone in the quarry
     */
    public Field.SkyStonePosition findSkyStoneLocation() {
        synchronized (skyStoneDetector) {
            /*To access the image: we need to iterate through the images of the frame object:*/
            VuforiaLocalizer.CloseableFrame vuforiaFrame; //takes the frame at the head of the queue
            try {
                //Match.log("Getting frame from vuForia");
                vuforiaFrame = vuforia.getFrameQueue().take();
                //Match.log("Got frame from vuForia");
                Image image = null;
                long numImages = vuforiaFrame.getNumImages();
                for (int i = 0; i < numImages; i++) {
                    Image checkedImage = vuforiaFrame.getImage(i);
                    int format = checkedImage.getFormat();
                    if (format == PIXEL_FORMAT.RGB565) {
                        image = checkedImage;
                        break;
                    }//if
                }//for
                if (image == null) {
                    Match.log("Unable to get image from vuForia camera out of " + numImages);
                    return Field.SkyStonePosition.TOP;
                }

                Bitmap bitmap = Bitmap.createBitmap(image.getWidth(), image.getHeight(), Bitmap.Config.RGB_565);
                //Match.log(image.getWidth() + ": " + image.getHeight());
                bitmap.copyPixelsFromBuffer(image.getPixels());
                return skyStoneDetector.getSkyStonePosition(bitmap);
            } catch (Exception e) {
                Match.log("Exception " + e + " in finding sky stone");
            }
            //default to top
            return Field.SkyStonePosition.TOP;
        }
    }

    public boolean isInitialized() {
            return isInitialized;
    }

    public String getLeftStoneValue() {
        return "" + skyStoneDetector.getValLeft();
    }
    public String getRightStoneValue() {
        return "" + skyStoneDetector.getValRight();
    }
    public String getMidStoneValue() {
        return "" + skyStoneDetector.getValMid();
    }
}
