package team492;

import frclib.FrcCANTalon;

public class Pickup
{
    private FrcCANTalon motor;

    public Pickup(Robot robot)
    {
        motor = new FrcCANTalon("PickupMotor", RobotInfo.CANID_PICKUP_MOTOR);
        motor.setBrakeModeEnabled(true);
    }

    public void setPickupPower(double power)
    {
        motor.set(power);
    }
}
