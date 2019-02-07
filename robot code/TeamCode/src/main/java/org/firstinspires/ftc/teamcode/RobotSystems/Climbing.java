package org.firstinspires.ftc.teamcode.RobotSystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Util.LogCreater;
import org.firstinspires.ftc.teamcode.Util.PIDController;

import static java.lang.Thread.sleep;

public class Climbing {
    public enum Angle {
        DOWN(10),
        DRIVE_POS(30),
        COLLECT(150),
        LAND(50),
        LANDFINAL(60),
        GO_TO_CLIMB(50),
        CLIMB(30),
        PUT(50);
        float pos;
        final double ticksPerDegree =93.588;

        public int getTicks() {
            return ((int) (ticksPerDegree * pos));
        }

        Angle(float ang) {
            this.pos = ang;
        }
    }

    public enum Height {
        DRIVE_POS(0),
        COLLECT(20),
        LAND(30),
        GO_TO_CLIMB(24),
        CLIMB(10),
        PUT(31);

        float pos;
        final double ticksPerCm = 134.2307;

        public int getTicks() {
            return ((int) (ticksPerCm * pos));
        }

        Height(float pos) {
            this.pos = pos;
        }
    }

    private final double LIFT_SPEED = 1;
    private final double HANG_OPEN_POS = 0.3;
    private final double HANG_CLOSE_POS = 0;
    private final double ANGLE_SPEED = 1;

    public DcMotor liftMotor;
    private Servo hangServo;
    public DcMotor angleMotorLeft;
    public DcMotor angleMotorRight;
    private OpMode opMode;
    private DigitalChannel liftTouch;
    private DigitalChannel angleTouch;
    private AnalogInput potentiometer;

    private boolean angleTouchIsPrevActive;
    private boolean angleTouchIsActive;
    private boolean liftTouchIsPrevActive;
    private boolean liftTouchIsActive;
    private double angleJoystickValue = 0;
    private double angleJoystickPrevValue = 0;
    private double liftJoystickValue = 0;
    private double liftJoystickPrevValue = 0;
    private double stopPIDTime;
    private PIDController anglePID;
    private final double KP = 1, KI = 0, KD = 0, TOLERANCE = 4;
    private LogCreater log;


    public void init(HardwareMap hardwareMap, OpMode opMode, LogCreater log) {
        this.log = log;
        init(hardwareMap, opMode);
    }
    public void init(HardwareMap hardwareMap, OpMode opMode) {
        this.opMode = opMode;
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        angleMotorLeft = hardwareMap.get(DcMotor.class, "angleMotorLeft");
        angleMotorRight = hardwareMap.get(DcMotor.class, "angleMotorRight");
        hangServo = hardwareMap.get(Servo.class, "hangServo");
        angleTouch = hardwareMap.get(DigitalChannel.class, "angle_touch");
        liftTouch = hardwareMap.get(DigitalChannel.class, "lift_touch");
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");
        anglePID = new PIDController(KP, KI, KD, TOLERANCE, opMode);

        angleMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        angleMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor.setPower(0);
        angleMotorLeft.setPower(0);
        angleMotorRight.setPower(0);

        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        angleMotorLeft.setDirection(DcMotor.Direction.FORWARD);
        angleMotorRight.setDirection(DcMotor.Direction.REVERSE);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        angleMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        angleMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftTouch.setMode(DigitalChannel.Mode.INPUT);
        angleTouch.setMode(DigitalChannel.Mode.INPUT);

        lockServo();
    }

    public void teleOpMotion(Gamepad operator) {
        angleTouchIsActive = !angleTouch.getState();
        liftTouchIsActive = !liftTouch.getState();

        angleJoystickValue = -operator.left_stick_y;
        liftJoystickValue = -operator.right_stick_y;

        if (angleTouchIsActive && !angleTouchIsPrevActive) {
            angleMotorLeft.setPower(0);
            angleMotorRight.setPower(0);
        }

        if (liftTouchIsActive && !liftTouchIsPrevActive) {
            liftMotor.setPower(0);
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (!liftMotor.isBusy() && liftMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            liftMotor.setPower(0);
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (anglePID.isRunning()) {
            if (anglePID.onTarget()) {
                angleMotorRight.setPower(0);
                angleMotorLeft.setPower(0);
                if (opMode.getRuntime() >= stopPIDTime) {
                    anglePID.stop();
                }
                anglePID.updateError(getAngle());
            } else {
                stopPIDTime = opMode.getRuntime() + 0.3;
                double output = anglePID.getOutput(getAngle());
                angleMotorLeft.setPower(output);
                angleMotorRight.setPower(output);
            }
        }


        if (operator.dpad_down) {
            moveLift(Height.DRIVE_POS);
            moveAngle(Angle.DRIVE_POS);
        }

        if (operator.dpad_left) {
            moveLift(Height.CLIMB);
            moveAngle(Angle.CLIMB);
        }

        if (operator.dpad_up) {
            moveLift(Height.GO_TO_CLIMB);
            moveAngle(Angle.GO_TO_CLIMB);
            openServo();
        }

        if (operator.right_trigger > 0.7) {
           moveLift(Height.COLLECT);
           moveAngle(Angle.COLLECT);
        }

        if (operator.left_trigger > 0.7) {
            moveLift(Height.PUT);
            moveAngle(Angle.PUT);
        }

        if (Math.abs(liftJoystickValue) > 0.1) {
            liftMoveManual(liftJoystickValue);
        } else if (Math.abs(liftJoystickPrevValue) > 0.1) {
            liftMoveManual(0);
        }

        if (Math.abs(angleJoystickValue) > 0.1) {
            angleMoveManual(angleJoystickValue);
        } else if (Math.abs(angleJoystickPrevValue) > 0.1) {
            angleMoveManual(0);
        }
        if (operator.a) {
            openServo();
        }

        if (operator.x) {
            lockServo();
        }


        opMode.telemetry.addLine("climbing: \n")
                .addData(" lift Position: ", liftMotor.getCurrentPosition() + "\n")
                .addData(" Angle Right Position: ", angleMotorRight.getCurrentPosition() + "\n")
                .addData(" Angle Left Position: ", angleMotorLeft.getCurrentPosition() + "\n")
                .addData("Servo position: ", hangServo.getPosition() + "\n")
                .addData("liftTouch: " , liftTouchIsActive)
                .addData(" angleTouch: ", angleTouchIsActive +"\n")
                .addData(" potentiometer: ", potentiometer.getVoltage())
                .addData("angle: ", getAngle());


        angleTouchIsPrevActive = angleTouchIsActive;
        liftTouchIsPrevActive = liftTouchIsActive;

        angleJoystickPrevValue = angleJoystickValue;
        liftJoystickPrevValue = liftJoystickValue;
    }


    public void moveLift(Height height) {
        liftMotor.setTargetPosition(height.getTicks());

        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotor.setPower(Math.abs(LIFT_SPEED));



    }

    private void liftMoveManual(double motorPower) {
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (liftTouchIsActive && motorPower < 0) {
            return;
        }
        liftMotor.setPower(motorPower * LIFT_SPEED);
    }

    public void moveAngle(Angle angle) {
        anglePID.reset(angle.pos, getAngle());
        double output = anglePID.getOutput(getAngle());
        angleMotorLeft.setPower(output);
        angleMotorRight.setPower(output);
    }

    public void moveAngleByPID(double target) {
        anglePID.reset(target, getAngle());
        while (!anglePID.onTarget()) {
            double output = anglePID.getOutput(getAngle());
            angleMotorRight.setPower(output);
            angleMotorLeft.setPower(output);
            opMode.telemetry.addData("error: " ,anglePID.getCurrentError())
                    .addData("output: " , output)
                    .addData("angle", getAngle());
            opMode.telemetry.update();
        }
        angleMotorRight.setPower(0);
        angleMotorLeft.setPower(0);
        while (((LinearOpMode) opMode).opModeIsActive()) {
            opMode.telemetry.addData("error: " ,anglePID.getCurrentError())
                    .addData("angle", getAngle());
            opMode.telemetry.update();
        }
    }
    private void angleMoveManual(double motorPower) {
        anglePID.stop();

        if(angleTouchIsActive && motorPower < 0) {
            return;
        }
        angleMotorLeft.setPower(motorPower * ANGLE_SPEED);
        angleMotorRight.setPower(motorPower * ANGLE_SPEED);
    }

    public void lockServo() {

        hangServo.setPosition(HANG_CLOSE_POS);

    }

    public void openServo() {
        hangServo.setPosition(HANG_OPEN_POS);
    }
    public void waitForFinish(DcMotor motor){
        while(motor.isBusy() && ((LinearOpMode) opMode).opModeIsActive()){
            opMode.telemetry.addData(motor.getDeviceName() + ": ",motor.getCurrentPosition());
            opMode.telemetry.update();
        }
    }
    public void waitForFinish(DcMotor motor1,DcMotor motor2){
        while(motor1.isBusy() && motor2.isBusy() && ((LinearOpMode) opMode).opModeIsActive()){
            opMode.telemetry.addData(motor1.getDeviceName() + ": ",motor1.getCurrentPosition());
            opMode.telemetry.addData(motor2.getDeviceName() + ": ",motor2.getCurrentPosition());
            opMode.telemetry.update();
        }
    }

    public void land() {
        moveAngleAuto(Climbing.Angle.LAND);
        moveliftAuto(Climbing.Height.LAND);
        moveAngleAuto(Climbing.Angle.LANDFINAL);
        openServo();
        ((LinearOpMode) opMode).sleep(600);
        moveAngleAuto(Angle.DOWN);
        moveliftAuto(Climbing.Height.CLIMB);

        angleMotorLeft.setPower(0);
        liftMotor.setPower(0);
    }

    public void moveliftAuto(Height height) {
        moveLift(height);
        waitForFinish(liftMotor);
        liftMotor.setPower(0);
    }

    public void moveAngleAuto(Angle angle) {
        anglePID.reset(angle.pos, getAngle());
        while (!anglePID.onTarget() && ((LinearOpMode)opMode).opModeIsActive()) {
            double output = anglePID.getOutput(getAngle());
            angleMotorRight.setPower(output);
            angleMotorLeft.setPower(output);
            opMode.telemetry.addData("error: ", anglePID.getCurrentError())
                    .addData("output: ", output)
                    .addData("angle", getAngle());
            opMode.telemetry.update();
        }
        angleMotorRight.setPower(0);
        angleMotorLeft.setPower(0);
    }
    public double getAngle() {
        return potentiometer.getVoltage() / 3.347 *270;
    }
}
