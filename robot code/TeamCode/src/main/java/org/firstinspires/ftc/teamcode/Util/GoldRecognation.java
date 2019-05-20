package org.firstinspires.ftc.teamcode.Util;
import android.support.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import javax.net.ssl.HandshakeCompletedEvent;


public class GoldRecognation {
    public enum MineralPos{
        LEFT,
        CENTER,
        RIGHT,
        UNKNOWN;
    }

    private OpMode opMode;
    private HardwareMap hardwareMap;
    private DcMotor leds;
    /**
     * this parameters discribe our labels that we want to find
     */
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private final double light = 0.05;

    private static final String VUFORIA_KEY = "AW/F0cP/////AAABmUpR4+dbt0Negw2nqaCH9Cw2gV4ZxuUmpeJMm7XOTdQVumthQcOeoS9qktHy4EvXtMAFoh7n5KeMiLMDqtKvd1TrbYNUy3f9ST3TMkH2hFYKB6oJMJPB8oelL9Bst/2XJBz0ycMMcKmSsIwyOqwOuHamAlwfT+o7VusfYmY7FPvnXuhn8obCeB5x0hhjwjsBuOz2wnx1us6N5y6on0rdc1DOzC2gI767QLVXAvPyJvMfgtZRGcfzFk0evuSVxrIPCRQBjQYK2s5SsBLZ4sEO4HelibKK5kg0lgT9P1uHSNa8SWX+4wpJm76dFw1nSL7YElieByOTXxycmOdhWaae5qb1lvYYmDiBNFiwfHRDkBRD";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    public TFObjectDetector tfod;

    public GoldRecognation(HardwareMap hardwareMap, OpMode opMode){
        this.hardwareMap = hardwareMap;
        this.opMode = opMode;
        this.leds = hardwareMap.get(DcMotor.class, "leds");
        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            opMode.telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        //parameters.cameraDirection = CameraDirection.FRONT;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
    public void stopTfod(){
        tfod.shutdown();
    }

    public void turnOnLeds() {
        leds.setPower(light);
    }

    public void turnOffLeds() {
        leds.setPower(0);
    }

    public MineralPos getGoldPos(LogCreater log) {
        turnOnLeds();
        if (((LinearOpMode) opMode).opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }
            double time = opMode.getRuntime();
            while (((LinearOpMode) opMode).opModeIsActive() && opMode.getRuntime() < time + 5) {
                List<Recognition> recognitionList = tfod.getUpdatedRecognitions();
                if (recognitionList != null) {
                    opMode.telemetry.addData("size", recognitionList.size());
                    if (log != null) {
                        log.writeLog("SamplingSize", recognitionList.size(), "");
                        for (Recognition recognition : recognitionList) {
                            log.writeLog("SamplingObject", recognition.getTop(),
                                    recognition.toString() + ", h: " + recognition.getHeight() + ", w: " + recognition.getWidth());
                        }
                    }
                    if (recognitionList.size() == 2) {
                        double goldTop = -1;
                        double silverTop = -1;
                        for (Recognition recognition : recognitionList) {
                            if (recognition.getLabel() == "Gold Mineral") {
                                goldTop = recognition.getTop();
                            } else if (silverTop == -1) {
                                silverTop = recognition.getTop();
                            } else {
                                return MineralPos.LEFT;
                            }
                        }
                        opMode.telemetry.addData("goldPos", goldTop + ",silverPos", silverTop);
                        if (goldTop != -1 && silverTop != -1) {
                            if (goldTop > silverTop) {
                                return MineralPos.CENTER;
                            } else return MineralPos.RIGHT;
                        }
                    }
                }
                opMode.telemetry.update();
            }
        }
        return MineralPos.UNKNOWN;
    }
    public void getGoldPosUsingCam() {
        List<Recognition> recognitionList = tfod.getUpdatedRecognitions();
        if(recognitionList != null){
            opMode.telemetry.addData("Size: ",recognitionList.size());
            opMode.telemetry.update();
            /*for(Recognition recognition : recognitionList){
                opMode.telemetry.addData("",recognition.toString());
                opMode.telemetry.update();
            }*/
        }

    }
    public MineralPos getGoldPosUsingCam(LogCreater log) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }
            List<Recognition> recognitionList = tfod.getUpdatedRecognitions();
            if (recognitionList != null) {
                opMode.telemetry.addData("size", recognitionList.size());
                opMode.telemetry.update();
                if (log != null) {
                    log.writeLog("SamplingSize", recognitionList.size(), "");
                    for (Recognition recognition : recognitionList) {
                        log.writeLog("SamplingObject", recognition.getTop(),
                        recognition.toString() + ", h: " + recognition.getHeight() + ", w: " + recognition.getWidth());
                    }
                }
                if (recognitionList.size() == 2) {
                    double goldLeft = -1;
                    double silverLeft = -1;
                    for (Recognition recognition : recognitionList) {
                        if (recognition.getLabel() == "Gold Mineral") {
                            goldLeft = recognition.getLeft();
                        } else if (silverLeft == -1) {
                            silverLeft = recognition.getLeft();
                        } else {
                            return MineralPos.RIGHT;
                        }
                    }
                    opMode.telemetry.addData("goldPos", goldLeft + ",silverPos", silverLeft);
                    opMode.telemetry.update();
                    if (goldLeft != -1 && silverLeft != -1) {
                        if (goldLeft > silverLeft) {
                            return MineralPos.CENTER;
                        } else return MineralPos.LEFT;
                    }
                }
            }
        return MineralPos.UNKNOWN;
    }
}
