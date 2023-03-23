package org.firstinspires.ftc.teamcode.THISIS10111.subsystems;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.THISIS10111.hardwaremaps.Robot10111;
import org.firstinspires.ftc.teamcode.THISIS10111.hardwaremaps.motors.CoolMotor101;

public class ArmObj extends SubsystemBase {

    private PIDFController armPID;
    public PIDFController armGet() {
        return armPID;
    }
    public PIDFController armSet(PIDFController liftPID) {
        this.armPID = liftPID;
        return this.armPID;
    }
    public double getPIDTarget() {
        return armPID.getTargetPosition();
    }

    private static ArmHeights armHeight;
    public ArmHeights getHeight() {
        return armHeight;
    }
    public void setHeight(ArmHeights armHeight) {
        this.armHeight = armHeight;
    }
    public double currentError;
    public boolean PIDControl = true;
    public double calculation = 0;
    double manualRunArm = 0;

    public ArmObj(final HardwareMap hwMap) {
        Robot10111 robot = Robot10111.getInstance();


        CommandScheduler.getInstance().registerSubsystem(ArmObj.this );

        robot.arm = new CoolMotor101(hwMap, "arm", 134.4);

        robot.arm.setInverted(true);




        armPID = new PIDFController(new PIDCoefficients(0.001, 0.001, 0));
        //armPID.setTargetPosition(0);
        this.setHeight(ArmHeights.ZERO);
        robot.intake = new SimpleServo(hwMap,"test",0,50);
        if (PIDControl == true) {


        }
        currentError = 0;
    }

    private double speed = 1;

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public double getSpeed() {
        return speed;
    }

    public void setRun(double manualArm){ this.manualRunArm = manualArm;}

    public void liftAutoController() {
       Robot10111 robot = Robot10111.getInstance();
        if(!PIDControl) robot.arm.set(this.manualRunArm);
        else {
            //robot.arm.runToPosition();
            switch (armHeight) {

                case ZERO:
                    armPID.setTargetPosition(0);
                    robot.arm.setTargetPosition(0);

                    break;
           /* case PICKUP:
                armPID.setTargetPosition(50);
                break;*/
                case DRIVE:
                    armPID.setTargetPosition(100);
                    robot.arm.setTargetPosition(100);
                    break;
                case BOTTOM:
                    armPID.setTargetPosition(100);
                    robot.arm.setTargetPosition(100);
                    //  Robot.getInstance().lift.setTarget(0);
                    break;
                case MIDDLE:
                    armPID.setTargetPosition(100);
                    robot.arm.setTargetPosition(100);
                    break;
                case TOP:
                    armPID.setTargetPosition(200);
                    // Robot.getInstance().lift.setTarget(100);
                    break;
            /*case CAP:
                armPID.setTargetPosition(100);
                break;*/
            }
            calculation = armPID.update(robot.arm.getEncoderCount());
            robot.arm.set(calculation * this.getSpeed());

            currentError = armPID.getLastError();
        }
    }

    @Override
    public void periodic() {
        liftAutoController();
    }

    public void openIntake() {
        Robot10111.getInstance().intake.setPosition(0.17);
    }

    public void closeIntake() {
        Robot10111.getInstance().intake.setPosition(0.4);
    }

    public void resetEncoderx() {
        Robot10111.getInstance().arm.resetEncoder();
    }



    public boolean isBusy() {
        Robot10111 robot = Robot10111.getInstance();
        boolean isBusy;
        if (robot.arm.busy())
            isBusy = true;
        else isBusy = false;
        return isBusy;
    }


    public void stop()
    {
        Robot10111 robot = Robot10111.getInstance();
        PIDControl = false;
        robot.arm.set(0);

    }
}

