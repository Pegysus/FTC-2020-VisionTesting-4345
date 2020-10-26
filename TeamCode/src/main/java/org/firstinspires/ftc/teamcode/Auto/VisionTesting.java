package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


@Autonomous(name = "Vision", group = "FTC 4345")
public class VisionTesting extends LinearOpMode {

    private static final String TFOD_MODEL = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY = "ARVhonL/////AAABmVtkHX43u0Wbgy8zlN4fkrVx+7eWkgEge8t4bqOsrlasrmX40oVuX0hbCdqzrBcl/eLUQ6sTzb1Jr9UI12ijg0aVE7saR9RPzTFbQAGW2rtBLCTFz3h7PdvD2WOeJBbEpZNeh7PXPVP/SIRv1JzwRJ5n9vANXpMwtxllDjY9V3/IPG3dW7PXZMgYmig5NMwz2r4wL8Ks0fdxNEfNH0+gWtzuoHbGHsSVMUq7zvG7BykSS/flHWjHR+d8OJeToa7f0PG+NVxnrnCVcAt3X614x7PVeIXqKNhcs/1i75dV3wu1aaixDjZ4fCaXXS1HsE0RsGa/WemoX5Do3BkhAghVGLuqsfoSNY7uAGpyMVaWz9Ky";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException {

        initVuforia();
        initTfod();

        if(tfod != null) {
            tfod.activate();
            tfod.setZoom(2.5, 1.78);
        }

        telemetry.addData(">", "thing is working");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {
            if(tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if(updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                    }
                    telemetry.update();
                }
            }
        }

        if(tfod != null) {
            tfod.shutdown();
        }

    }

    private void initVuforia() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "CameraGoBrrr");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }

    private void initTfod() {

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

    }

}
