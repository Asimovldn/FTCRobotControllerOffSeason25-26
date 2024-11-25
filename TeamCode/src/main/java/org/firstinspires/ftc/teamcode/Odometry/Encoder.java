package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Encoder
{
    public static int ENCODER_TICKS = 2000;
    public static double WHEEL_DIAMETER = 4.8;

    public static double TICKS2CM = (WHEEL_DIAMETER * 3.1415) / ENCODER_TICKS;

    int encoderMultiplier = 1;

    DcMotorEx encoderMotorPort;

    public Encoder(DcMotorEx encoderMotorPort)
    {
        this.encoderMotorPort = encoderMotorPort;
        DcMotor.RunMode mode = encoderMotorPort.getMode();
        encoderMotorPort.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        encoderMotorPort.setMode(mode);
    }

    public int getPosition()
    {
        return encoderMotorPort.getCurrentPosition() * encoderMultiplier;

    }

    public double getAngularVelocity(AngleUnit angleUnit)
    {
        return encoderMotorPort.getVelocity(angleUnit) * encoderMultiplier;
    }

    public double getAngularVelocity()
    {
        return encoderMotorPort.getVelocity() * encoderMultiplier;
    }

    public void reverseEncoder(boolean reversed)
    {
        encoderMultiplier = reversed ? -1 : 1;
    }






}
