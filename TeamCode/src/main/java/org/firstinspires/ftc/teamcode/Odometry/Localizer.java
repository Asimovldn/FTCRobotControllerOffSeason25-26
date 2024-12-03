package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Auxiliar.DriveConstants;
import org.firstinspires.ftc.teamcode.Auxiliar.Vector2D;

public class Localizer
{
    Encoder parallelEncoder;
    Encoder lateralEncoder;

    Vector2D initialPosition = new Vector2D(0,0);
    double initialAngle = 0;

    IMU imu;

    // Paralela, Lateral
    int[] encoderTicks;
    int[] lastEncoderTicks = new int[] {0,0};

    double lastAngle = 0;
    public double currentAngle;

    public double dAngle = 0;

    Vector2D currentPosition = initialPosition;

    public Localizer(HardwareMap hardwareMap)
    {
        // MUDAR AS PORTAS DOS ENCODERES
        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class,
                "left_front_drive"));
        lateralEncoder = new Encoder(hardwareMap.get(DcMotorEx.class,
                "right_back_drive"));

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(DriveConstants.imuParameters);
        imu.resetYaw();
    }

    public void update()
    {

        encoderTicks = getEncoderTicks();

        double parallelDistanceToCenter = 25;
        double lateralDistanceToCenter = 21;

        double parallelDisplacement = (encoderTicks[0] - lastEncoderTicks[0]) * Encoder.TICKS2CM;
        double lateralDisplacement = (encoderTicks[1] - lastEncoderTicks[1]) * Encoder.TICKS2CM;

        currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double angleDisplacement = currentAngle - lastAngle;
        angleDisplacement = (angleDisplacement + Math.PI) % (2 * Math.PI) - Math.PI;
        //angleDisplacement = normalizeAngle(angleDisplacement);

        dAngle = angleDisplacement;

        parallelDisplacement -= parallelDistanceToCenter * angleDisplacement;
        lateralDisplacement -= -lateralDistanceToCenter * angleDisplacement;

        Vector2D robotDisplacement = new Vector2D(0,0);

        robotDisplacement.x += Math.cos(angleDisplacement) * parallelDisplacement - Math.sin(angleDisplacement) * lateralDisplacement;
        robotDisplacement.y += Math.sin(angleDisplacement) * parallelDisplacement + Math.cos(angleDisplacement) * lateralDisplacement;

        currentPosition.x += robotDisplacement.x;
        currentPosition.y += robotDisplacement.y;

        lastEncoderTicks = encoderTicks;
        lastAngle = currentAngle;

    }

    public Vector2D getCurrentPosition()
    {
        return currentPosition;
    }

    int[] getEncoderTicks()
    {
        return new int[] {parallelEncoder.getPosition(), lateralEncoder.getPosition()};
    }

    public double normalizeAngle(double angle)
    {
        return angle < 0 ? angle + 360 : angle;
    }



    

}
