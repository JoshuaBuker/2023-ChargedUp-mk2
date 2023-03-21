package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public class Superstructure {
    private final PneumaticHub pneumatics = new PneumaticHub(Constants.PNEUMATICS_HUB_ID);
    private final  PowerDistribution pdp = new PowerDistribution(Constants.POWER_DISTRIBUTION_ID, ModuleType.kRev);

    private UsbCamera cam;

    public Superstructure() {
        pneumatics.enableCompressorDigital();
        pdp.clearStickyFaults();

        cam = CameraServer.startAutomaticCapture();
        cam.setResolution(640, 480);
        
    }
}
