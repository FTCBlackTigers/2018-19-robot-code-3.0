package org.firstinspires.ftc.teamcode.RobotSystems;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.Util.LogCreater;

public class Intake {
    enum Minerals{
        GOLD, SILVER, UNKNOWN;
    }

    private final double COLLECTION_SPEED = 0.8;
    private final double RELEASE_SPEED = 0.5;
    private final double LEFT_SERVO_OPEN_POS = 0.8;
    private final double RIGHT_SERVO_OPEN_POS = 0;
    private final double LEFT_SERVO_CLOSE_POS = 0.20;
    private final double RIGHT_SERVO_CLOSE_POS = 0.7;

    private OpMode opMode;

    private DcMotor collectMotor;
    public Servo leftServo, rightServo;
    private ColorSensor leftColorSensor, rightColorSensor;
    private Minerals searchMineral;

    private boolean collectModeIsPrevActive;
    private boolean releaseModeIsPrevActive;
    private boolean releaseModeIsActive;
    private boolean collectModeIsActive;
    private boolean injecktIsActive;
    private boolean injecktIsPrevActive;
    private LogCreater log;
    private Minerals leftMineral = Minerals.UNKNOWN;
    private Minerals rightMineral = Minerals.UNKNOWN;
    private double leftRecognationCounter = 0;
    private double rightRecognationCounter = 0;
    private double timeToOpenRight = -1;

    public void init(HardwareMap hardwareMap, OpMode opMode, LogCreater log){
        this.log = log;
        init(hardwareMap, opMode);
    }
    public void init(HardwareMap hardwareMap, OpMode opMode) {
        this.opMode = opMode;
        collectMotor = hardwareMap.get(DcMotor.class, "collectMotor");
        collectMotor.setDirection(DcMotor.Direction.FORWARD);
        collectMotor.setPower(0);
        collectMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftColorSensor = hardwareMap.get(ColorSensor.class, "leftColorSensor");
        rightColorSensor = hardwareMap.get(ColorSensor.class, "rightColorSensor");
        searchMineral = Minerals.GOLD;

        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        closeLeftGate();
        closeRightGate();
        if (leftColorSensor instanceof SwitchableLight && rightColorSensor instanceof SwitchableLight) {
            ((SwitchableLight) leftColorSensor).enableLight(false);
            ((SwitchableLight) rightColorSensor).enableLight(false);
        }
    }

    public void teleOpMotion(Gamepad driver, Gamepad operator) {

        releaseModeIsActive = driver.left_bumper || operator.left_bumper;
        collectModeIsActive = driver.right_bumper || operator.right_bumper;
        injecktIsActive = driver.b;

        if(collectModeIsActive) {
            this.collect();
        }
        else if (releaseModeIsActive) {
            this.release();
        }

        if(collectModeIsPrevActive && !collectModeIsActive) {
            closeRightGate();
            closeLeftGate();
            this.stopMotor();
        }

        if(releaseModeIsPrevActive && !releaseModeIsActive) {
            this.stopMotor();
            closeRightGate();
            closeLeftGate();
            timeToOpenRight = -1;
        }

        if(operator.y) {
            setSearchMineral(Minerals.GOLD);
        }

        if(operator.b) {
            setSearchMineral(Minerals.SILVER);
        }

        if (injecktIsActive) {
            this.injackt();

        }

        if(injecktIsPrevActive && !injecktIsActive) {
            closeRightGate();
            closeLeftGate();
            this.stopMotor();
        }

        opMode.telemetry.addLine("intake: \n" )
                .addData("collectMotorPower: ", collectMotor.getPower()+"\n")
                .addData("rightServoPos: ", rightServo.getPosition())
                .addData("leftServoPos: ", leftServo.getPosition()+"\n")
                .addData("search mineral: ", searchMineral+"\n")
                .addData("releaseModeIsPrevActive: ", releaseModeIsPrevActive)
                .addData("collectModeIsPrevActive: ", collectModeIsPrevActive +"\n")
        .addData("leftMineral", leftMineral).addData("rightMineral", rightMineral);

        releaseModeIsPrevActive = releaseModeIsActive;
        collectModeIsPrevActive = collectModeIsActive;
        injecktIsPrevActive = injecktIsActive;
    }

    private void openLeftGate() {
        leftServo.setPosition(LEFT_SERVO_OPEN_POS);
        leftMineral = Minerals.UNKNOWN;
        leftRecognationCounter = 0;
    }

    private void openRightGate() {
        rightServo.setPosition(RIGHT_SERVO_OPEN_POS);
        rightMineral = Minerals.UNKNOWN;
        rightRecognationCounter = 0;
    }

    private void closeLeftGate() {
        leftServo.setPosition(LEFT_SERVO_CLOSE_POS);
    }

    private void closeRightGate() {
        rightServo.setPosition(RIGHT_SERVO_CLOSE_POS);
    }

    public void release(){
        if (timeToOpenRight == -1) {
            timeToOpenRight = opMode.getRuntime();
        }
        openLeftGate();
        if (timeToOpenRight + 1 <= opMode.getRuntime()) {
            openRightGate();
        }
        collectMotor.setPower(RELEASE_SPEED);
    }


   public void collect() {
        collectMotor.setPower(COLLECTION_SPEED);
        this.leftOutput();
        this.rightOutput();
    }
    public void injackt() {
        collectMotor.setPower(-COLLECTION_SPEED);
        openRightGate();
        openLeftGate();
    }

    private Minerals getLeftMineral() {
        float[] hsvValues = new float[3];
        Color.RGBToHSV(leftColorSensor.red() * 255, leftColorSensor.green() * 255, leftColorSensor.blue()* 255, hsvValues);
        opMode.telemetry.addLine("leftColorSensor: ").addData("Value: ", hsvValues[2]);
        if (log != null) {
            log.writeLog("leftColorSensor", hsvValues[2], "searchMineral: " + searchMineral);
        }


        if(hsvValues[2] >= 700) {
            return Minerals.SILVER;
        }
        if (hsvValues[2] >= 150 && hsvValues[2] <= 400) {
            return Minerals.GOLD;
        }
        return Minerals.UNKNOWN;
    }

    private Minerals getRightMineral() {
        float[] hsvValues = new float[3];
        Color.RGBToHSV(rightColorSensor.red() * 255, rightColorSensor.green() * 255, rightColorSensor.blue()* 255, hsvValues);
        opMode.telemetry.addLine("rightColorSensor: ").addData("Value: ", hsvValues[2]);
       if (log != null) {
           log.writeLog("rightColorSensor", hsvValues[2], "searchMineral:" + searchMineral);
       }

        if(hsvValues[2] >= 1700) { // 1550
            return Minerals.SILVER;
        }
        if (hsvValues[2] >= 350 && hsvValues[2] <= 800) { // 300, 550
            return Minerals.GOLD;
        }
        return Minerals.UNKNOWN;
    }


    private void leftOutput() {
        if (leftMineral != searchMineral) {
            Minerals imat = getLeftMineral();
            if (imat == Minerals.UNKNOWN) {
                closeLeftGate();
            } else if (imat != searchMineral) {
                openLeftGate();
            } else {
                leftRecognationCounter++;
                if (leftRecognationCounter >= 10) {
                    leftMineral = searchMineral;
                }

            }
        }
    }

    private void rightOutput() {
        if (rightMineral != searchMineral) {
            Minerals imat = getRightMineral();
            if (imat == Minerals.UNKNOWN) {
                closeRightGate();
            } else if (imat != searchMineral) {
                openRightGate();
            } else {
                rightRecognationCounter++;
                if (rightRecognationCounter >= 10) {
                    rightMineral = searchMineral;
                }
            }
        }
    }

    public void stopMotor(){
        collectMotor.setPower(0);
        closeRightGate();
        closeLeftGate();
    }

    public void setSearchMineral(Minerals mineral){
        this.searchMineral = mineral;
        rightMineral = Minerals.UNKNOWN;
        leftMineral = Minerals.UNKNOWN;
        leftRecognationCounter = 0;
        rightRecognationCounter = 0;
    }

}



