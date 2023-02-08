package org.firstinspires.ftc.teamcode.action;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.other.secrets;

import java.util.List;
import java.util.Objects;

/** This is an interface for this year's image detector. */
public class imageDetectionTF {
    // CONSTRUCT
    static org.firstinspires.ftc.teamcode.other.secrets secrets = new secrets(); // Imports Vuforia key from local file
    // DECLARE NULL
    HardwareMap hardwareMap;
    private VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;
    private static String label;
    // DECLARE CUSTOM
    private static final String DEFAULT_TFOD_MODEL_ASSET = "PowerPlay.tflite"; // Default model
    private static final String TFOD_MODEL_FILE_VERSION2  = "/sdcard/FIRST/tflitemodels/model_FTC8417_V2.tflite"; // Custom models
    private static final String TFOD_MODEL_FILE_VERSION4  = "/sdcard/FIRST/tflitemodels/model_FTC8417_V4.tflite";
    private static final String VUFORIA_KEY = secrets.REPLACE_ME_WITH_YOUR_OWN_VUFORIA_KEY; // Vuforia key
    private static final String[] DEFAULT_LABELS = { // Labels to use for the default model
            "Bolt",
            "Light",
            "Panel"
    };
    private static final String[] CUSTOM_LABELS = { // Labels to use for the custom model
            "Handsaw",
            "Robot",
            "Turtle"
    };
    private static float confidence = -1;

    // METHODS
    /** Initializes the image detector.
     * @param opMode If you are constructing from an Auto or TeleOp, type in "this" without the quotation marks.
     */
    public void init(@NonNull OpMode opMode){
        hardwareMap = opMode.hardwareMap;
    }

    /** Initializes Vuforia */
    public void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /** Initializes TensorFlow */
    public void initTfod(String teamColor) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.70f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        /* Use loadModelFromAsset() if the model is given to you by FTC
        Use loadModelFromFile() if you have downloaded a custom model to the robot. */
        if(Objects.equals(teamColor, "Default")) {
            tfod.loadModelFromAsset(DEFAULT_TFOD_MODEL_ASSET, DEFAULT_LABELS);
        } else if (Objects.equals(teamColor, "Blue")) {
            tfod.loadModelFromFile(TFOD_MODEL_FILE_VERSION2, CUSTOM_LABELS);
        } else {
            tfod.loadModelFromFile(TFOD_MODEL_FILE_VERSION4, CUSTOM_LABELS);
        }

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }
    }

    /** Returns the label of a new image, if any. */
    public String imageReturn(){
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getConfidence() > confidence) {
                        confidence = recognition.getConfidence();
                        label = recognition.getLabel();
                    }
                }
                return label;
            }
        }
        return null;
    }
}