package org.firstinspires.ftc.teamcode.RobotSystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Util.GlobalVariebels;
import org.firstinspires.ftc.teamcode.Util.LogCreater;
import org.firstinspires.ftc.teamcode.Util.PIDController;

import static java.lang.Thread.sleep;

public class Climbing {
    public enum Angle {
        DOWN(10),
        DRIVE_POS(30),
        COLLECT(160),
        LAND(50),
        LANDFINAL(60),
        GO_TO_CLIMB(60),
        CLIMB(30),
        PUT(61);
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
        COLLECT(15),
        LAND(30),
        GO_TO_CLIMB(29),
        CLIMB(10),
        PUT(36);

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
    private final double HANG_OPEN_POS = 0.25;
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
    private final double KP = 0.5, KI = 0, KD = 0, TOLERANCE = 4;
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
            GlobalVariebels.liftPosEndAuto = 0;
            if (log != null) {
                log.writeLog("liftMotor", liftMotor.getCurrentPosition(), "reset encoder");
            }
        }

        if (!liftMotor.isBusy() && liftMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            liftMotor.setPower(0);
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (log != null) {
                log.writeLog("liftMotor", liftMotor.getCurrentPosition(), "arrived to target " + liftMotor.getTargetPosition());
            }
        } else if(liftMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            if (log != null) {
                log.writeLog("liftMotor", liftMotor.getCurrentPosition(), "target: " + liftMotor.getTargetPosition());
            }
        }
        if (anglePID.isRunning()) {
            if (anglePID.onTarget()) {
                angleMotorRight.setPower(0);
                angleMotorLeft.setPower(0);
                if (opMode.getRuntime() >= stopPIDTime) {
                    anglePID.stop();
                    if (log != null) {
                        log.writeLog("angleMotor", getAngle(), "arrived to target " + anglePID.getSetpoint());
                    }
                }
                anglePID.updateError(getAngle());
            } else {
                stopPIDTime = opMode.getRuntime() + 0.3;
                double output = anglePID.getOutput(getAngle());
                if(IsLiftInDeadZone()){
                    output *= 0.1;
                    opMode.telemetry.addData("output: " , output);
                }
                angleMotorLeft.setPower(output);
                angleMotorRight.setPower(output);
                if (log != null) {
                    log.writeLog("angleMotor", getAngle(), "target: " + anglePID.getSetpoint());
                }
            }
        }


        if (operator.dpad_down) {
            moveLift(Height.DRIVE_POS);
            moveAngle(Angle.DRIVE_POS);
            if (log != null) {
                log.writeLog("liftMotor", liftMotor.getCurrentPosition(), "command to move to drive pos " + Height.DRIVE_POS.getTicks());
                log.writeLog("angleMotor", getAngle(), "command to move to drive pos " + Angle.DRIVE_POS.pos);
            }
        }

        if (operator.dpad_left) {
            moveLift(Height.CLIMB);
            moveAngle(Angle.CLIMB);
            if (log != null) {
                log.writeLog("liftMotor", liftMotor.getCurrentPosition(), "command to move to climb " + Height.CLIMB.getTicks());
                log.writeLog("angleMotor", getAngle(), "command to move to climb " + Angle.CLIMB.pos);
            }
        }

        if (operator.dpad_up) {
            moveLift(Height.GO_TO_CLIMB);
            moveAngle(Angle.GO_TO_CLIMB);
            openServo();
            if (log != null) {
                log.writeLog("liftMotor", liftMotor.getCurrentPosition(), "command to move to go to climb " + Height.GO_TO_CLIMB.getTicks());
                log.writeLog("angleMotor", getAngle(), "command to move to go to climb " + Angle.GO_TO_CLIMB.pos);
            }
        }

        if (operator.right_trigger > 0.7) {
           moveLift(Height.COLLECT);
           moveAngle(Angle.COLLECT);
            if (log != null) {
                log.writeLog("liftMotor", liftMotor.getCurrentPosition(), "command to move to collect " + Height.COLLECT.getTicks());
                log.writeLog("angleMotor", getAngle(), "command to move to collect " + Angle.COLLECT.pos);
            }
        }

        if (operator.left_trigger > 0.7) {
            moveLift(Height.PUT);
            moveAngle(Angle.PUT);
            if (log != null) {
                log.writeLog("liftMotor", liftMotor.getCurrentPosition(), "command to move to put " + Height.PUT.getTicks());
                log.writeLog("angleMotor", getAngle(), "command to move to put " + Angle.PUT.pos);
            }
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
                .addData("angle: ", getAngle())
                .addData("liftPosEndAuto", GlobalVariebels.liftPosEndAuto);


        angleTouchIsPrevActive = angleTouchIsActive;
        liftTouchIsPrevActive = liftTouchIsActive;

        angleJoystickPrevValue = angleJoystickValue;
        liftJoystickPrevValue = liftJoystickValue;
    }


    public void moveLift(Height height) {
        liftMotor.setTargetPosition(height.getTicks() - GlobalVariebels.liftPosEndAuto);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(Math.abs(LIFT_SPEED));
        if(height == Height.GO_TO_CLIMB && IsliftInTarget(Height.GO_TO_CLIMB)){
            liftMotor.setPower(0.1); //In Order to anchor the motor and make him no fall
        }
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
        if(IsLiftInDeadZone()){
            output *= 0.1;
            opMode.telemetry.addData("output: " , output);
        }
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

    public void land() {
        //moveAngleAuto(Climbing.Angle.LAND);
        //moveLiftAuto(Climbing.Height.LAND);
        //moveAngleAuto(Climbing.Angle.LANDFINAL);
        moveAngleAndHeight(Climbing.Angle.LANDFINAL, Climbing.Height.LAND);
        openServo();
        ((LinearOpMode) opMode).sleep(600);
        //moveLiftAuto(Climbing.Height.CLIMB);
        //moveAngleAuto(Angle.DOWN);
        moveAngleAndHeight(Angle.DOWN, Height.CLIMB);

        angleMotorLeft.setPower(0);
        liftMotor.setPower(0);
    }

    public void moveLiftAuto(Height height) {
        moveLift(height);
        while(liftMotor.isBusy() && ((LinearOpMode) opMode).opModeIsActive()) {
            opMode.telemetry.addData("liftMotor: ", liftMotor.getCurrentPosition());
            opMode.telemetry.update();
            if (log != null) {
                log.writeLog("lift", liftMotor.getCurrentPosition(), "target: " + height.getTicks());
            }
        }
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
            if (log != null) {
                log.writeLog("angle", getAngle(), "target: " + angle.pos);
            }
        }
        angleMotorRight.setPower(0);
        angleMotorLeft.setPower(0);
    }

    public  void moveAngleAndHeight(Angle angle, Height height){
        anglePID.reset(angle.pos, getAngle());

        liftMotor.setTargetPosition(height.getTicks());
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(Math.abs(LIFT_SPEED));

        while (!anglePID.onTarget()){
            double output = anglePID.getOutput(getAngle());
            angleMotorRight.setPower(output);
            angleMotorLeft.setPower(output);
            opMode.telemetry.addLine("Angle \n")
                    .addData("error: ", anglePID.getCurrentError())
                    .addData("output: ", output)
                    .addData("angle", getAngle() + "\n");

            opMode.telemetry.addLine("Lift \n")
                    .addData("position: ", liftMotor.getCurrentPosition());
            opMode.telemetry.update();

            if(!liftMotor.isBusy()){
                liftMotor.setPower(0);
                liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (log != null) {
                log.writeLog("angle", getAngle(), "target: " + angle.pos);
                log.writeLog("lift", liftMotor.getCurrentPosition(), "target: " + height.getTicks());
            }
        }
        angleMotorLeft.setPower(0);
        angleMotorRight.setPower(0);

        while(liftMotor.isBusy()){
            opMode.telemetry.addLine("Lift \n")
                    .addData("position: ", liftMotor.getCurrentPosition());
            opMode.telemetry.update();
            if (log != null) {
                log.writeLog("lift", liftMotor.getCurrentPosition(), "target: " + height.getTicks());
            }
        }

        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getAngle() {
        return potentiometer.getVoltage() / 3.347 *270;
    }

    public void setOpMode(OpMode opMode) {
        this.opMode = opMode;
    }

    public void setLog(LogCreater log) {
        this.log = log;
    }
    public boolean IsLiftInDeadZone(){
        return getAngle()>49 && getAngle()<80;
    }
    public boolean IsliftInTarget(Height height){
        if(liftMotor.getTargetPosition() >= height.getTicks()+200 && liftMotor.getTargetPosition() <= height.getTicks()-200){
            return true;
        } else return false;
    }
}
