package org.firstinspires.ftc.teamcode.RobotSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Util.LogCreater;

public class Intake {

    private final double COLLECTION_SPEED = 1;
    private final double INJECT_SPEED = 0.5;
    private final double RELEASE_SPEED = 0.75;
    private final double LEFT_SERVO_OPEN_POS = 0.8;
    private final double RIGHT_SERVO_OPEN_POS = 0;
    private final double LEFT_SERVO_CLOSE_POS = 0.20;
    private final double RIGHT_SERVO_CLOSE_POS = 0.7;

    private OpMode opMode;
    private LogCreater log;

    private DcMotor collectMotor;
    public Servo leftServo, rightServo;

    private boolean collectModeIsPrevActive;
    private boolean releaseModeIsPrevActive;
    private boolean releaseModeIsActive;
    private boolean collectModeIsActive;
    private boolean injecktIsActive;
    private boolean injecktIsPrevActive;

    private double timeToOpenRight = -1;

    public void init(HardwareMap hardwareMap, OpMode opMode, LogCreater log){
        this.log = log;
        init(hardwareMap, opMode);
    }
    public void init(HardwareMap hardwareMap, OpMode opMode) {
        this.opMode = opMode;
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

        collectMotor = hardwareMap.get(DcMotor.class, "collectMotor");
        collectMotor.setDirection(DcMotor.Direction.FORWARD);
        collectMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stop();
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
            this.stop();
        }

        if(releaseModeIsPrevActive && !releaseModeIsActive) {
            this.stop();
            closeRightGate();
            closeLeftGate();
            timeToOpenRight = -1;
        }

        if (injecktIsActive) {
            this.injackt();

        }

        if(injecktIsPrevActive && !injecktIsActive) {
            closeRightGate();
            closeLeftGate();
            this.stop();
        }
        if(driver.y){
            openLeftGate();
            openRightGate();
        }

        opMode.telemetry.addLine("intake: \n" )
                .addData("collectMotorPower: ", collectMotor.getPower()+"\n")
                .addData("rightServoPos: ", rightServo.getPosition())
                .addData("leftServoPos: ", leftServo.getPosition()+"\n");

        releaseModeIsPrevActive = releaseModeIsActive;
        collectModeIsPrevActive = collectModeIsActive;
        injecktIsPrevActive = injecktIsActive;
    }

    public void openLeftGate() {
        leftServo.setPosition(LEFT_SERVO_OPEN_POS);
    }

    public void openRightGate() {
        rightServo.setPosition(RIGHT_SERVO_OPEN_POS);
    }

    public void closeLeftGate() {
        leftServo.setPosition(LEFT_SERVO_CLOSE_POS);
    }

    public void closeRightGate() {
        rightServo.setPosition(RIGHT_SERVO_CLOSE_POS);
    }


    public void collect() {
        collectMotor.setPower(COLLECTION_SPEED);
    }

    public void release(){
        if(timeToOpenRight == -1) {
            timeToOpenRight = opMode.getRuntime();
        }
        openLeftGate();
        if(opMode.getRuntime() >= timeToOpenRight + 0.5) {
            openRightGate();
        }
        collectMotor.setPower(RELEASE_SPEED);
    }

    public void injackt() {
        collectMotor.setPower(-INJECT_SPEED);
        openRightGate();
        openLeftGate();
    }
    public void stop(){
        collectMotor.setPower(0);
        closeRightGate();
        closeLeftGate();
    }

    public void setOpMode(OpMode opMode) {
        this.opMode = opMode;
    }

    public void setLog(LogCreater log) {
        this.log = log;
    }
}



