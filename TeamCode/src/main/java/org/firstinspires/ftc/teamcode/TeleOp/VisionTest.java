package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


@TeleOp(name = "Vision go brrr", group = "FTC 4345")
public class VisionTest extends LinearOpMode {

    /*
     * Objects
     * vuforia will be initializing and creating the camera (camera object0, tfod is the object for
     * the tfod model.
     */
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    // tfod object detection system
    // TFOD_MODEL is just specifying the model, first and second element variables are just the names (Quad = 4 stack and Single = 1 stack)
    private static final String TFOD_MODEL = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY = "ARVhonL/////AAABmVtkHX43u0Wbgy8zlN4fkrVx+7eWkgEge8t4bqOsrlasrmX40oVuX0hbCdqzrBcl/eLUQ6sTzb1Jr9UI12ijg0aVE7saR9RPzTFbQAGW2rtBLCTFz3h7PdvD2WOeJBbEpZNeh7PXPVP/SIRv1JzwRJ5n9vANXpMwtxllDjY9V3/IPG3dW7PXZMgYmig5NMwz2r4wL8Ks0fdxNEfNH0+gWtzuoHbGHsSVMUq7zvG7BykSS/flHWjHR+d8OJeToa7f0PG+NVxnrnCVcAt3X614x7PVeIXqKNhcs/1i75dV3wu1aaixDjZ4fCaXXS1HsE0RsGa/WemoX5Do3BkhAghVGLuqsfoSNY7uAGpyMVaWz9Ky";

    // vuforia field localization
    // allTrackables is a list that will hold all the images that are trackable from the vuforia package
    private List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    // targetUltimateGoal is used to import the trackable images
    private VuforiaTrackables targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
    // lastLocation is the previous location that a target (field localization) was seen
    private OpenGLMatrix lastLocation = null;


    /*
     * CAMERA_DIRECTION and PHONE_IS_PORTRAIT and used to reorient the camera to ensure the transformation matrix is accurate.
     * For webcams, CAMERA_DIRECTION = BACk and PHONE_IS_PORTRAIT = false should be correct values,
     * but it can change based off of where the camera is locaatred on the bot
     */
    private static final VuforiaLocalizer.CameraDirection CAMERA_DIRECTION = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

    // phone rotation/displacement matrix for field localization
    private boolean targetVisible = false;
    private float cameraXRotate = 0; // 0 degrees rotated in the x direction
    private float cameraYRotate = 0; // 0 degrees rotated in the y direction
    private float cameraZRotate = 0; // 0 degrees rotated in the z direction

    private final float CAMERA_X_DISPLACEMENT = 4.0f * mmPerInch; // 4 inches in front of bot center
    private final float CAMERA_Y_DISPLACEMENT = 0;                // 0 inches from center
    private final float CAMERA_Z_DISPLACEMENT = 8.0f * mmPerInch; // 8 inches above ground

    // conversion factors (inches to mm and other important variables to convert
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = 6 * mmPerInch;
    private static final float halfField = 72 * mmPerInch;
    private static final float fourthField = 36 * mmPerInch;

    @Override
    public void runOpMode() throws InterruptedException {

        // init camera + field localization
        initVuforia();
        // create object detection
        initTfod();

        // Activates tfod model (zoom is to ensure that the model can be more accurate in detecting)
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(2, 1.78); // values can change
        }
        // Activates field localization
        targetsUltimateGoal.activate();

        waitForStart();

        while (opModeIsActive()) {

            // object detection
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions(); // list of recognized objects (4-stack of disks or 1-stack of disc)
                if (updatedRecognitions != null) {
                    // just printing out all of the recognized objects and what type they are
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                    }
                }
            }

            // this is just to wait for a bit to make sure both object detection and field localization is runnign at the same time
            sleep(3000);

            // field localization
            targetVisible = false; // initially no targets visible
            for(VuforiaTrackable trackable: allTrackables) { // runs through all images
                // if statement checks if the current image is visible in the camera
                if(((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName()); // if visible, print out data of it
                    targetVisible = true;

                    // use the image to get the location of the robot, and if you can update the location (if robot mobed)
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if(robotLocationTransform != null)
                        lastLocation = robotLocationTransform;
                    break;
                }
            }

            // output if robot can see target
            if(targetVisible) {
                // translate the robot
                VectorF translation = lastLocation.getTranslation(); // translation matrix of robot
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0)/mmPerInch, translation.get(1)/mmPerInch, translation.get(2)/mmPerInch);

                // rotation of robot
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES); /// rotation matrix of robot
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f",
                        rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            } else {
                telemetry.addData("Visible Target", "none");
            }

            telemetry.update();
        }

        if (tfod != null) {
            tfod.shutdown();
        }
        targetsUltimateGoal.deactivate();

    }

    private void initTfod() { // Initialize tfod model

        // retrieveing specific package and parameters for the camera
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f; // if 80% confident, then the detection is correct
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

    }

    private void initVuforia() { // Initialize camera + setup field localization
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "CameraGoBrrr");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // input vuforia images
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        // create trackables
        allTrackables.addAll(targetsUltimateGoal); // Uses the images that you get from code above

        /*
         * Create transformation matrix for the location of the photos
         * You need to create a transformation matrix for all of these images to accurately know
         * the correct location of your current bot. The transformation matrxi that we are using is
         * a translational and rotational one. The translation occurs since it's some distance away
         * from the origin and rotation occurs since depending on which side of the wall its on,
         * the image will be rotated in different orientations. Knowing the transformation matrix
         * (both the translational and rotational) of the images and the matrix of your robot
         * (created later) will allow you to figure out the standard translation or rotation
         * matrix, or the matrix defining the distance and angle your robot is from the tracking
         * target
         */
        // Perimeter targets
        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        // Tower goal targets
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, fourthField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -fourthField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        // Phone displacement (this is where the 2 global variables are used)
        if (CAMERA_DIRECTION == BACK)
            cameraYRotate = -90;
        else
            cameraYRotate = 90;

        if (PHONE_IS_PORTRAIT)
            cameraXRotate = 90;

        /*
         * setup phone transformation matrix
         * This case, the global variables can be changed based off of the location that the camera
         * is placed. But the difference in this matrix is the order of euler angles (roll-pitch-yaw
         * or XYZ angles). We use YZX instead of the normal XYZ because of the initial orientation of
         * the camera.
         */
        OpenGLMatrix cameraLocation = OpenGLMatrix
                .translation(CAMERA_X_DISPLACEMENT, CAMERA_Y_DISPLACEMENT, CAMERA_Z_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, cameraYRotate, cameraZRotate, cameraXRotate));

        // Get listeners to make sure you can track the vuforia targets
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(cameraLocation, parameters.cameraDirection);
        }
    }
}
