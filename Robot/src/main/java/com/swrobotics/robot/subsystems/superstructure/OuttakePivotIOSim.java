package com.swrobotics.robot.subsystems.superstructure;

import com.swrobotics.lib.utils.MathUtil;
import com.swrobotics.robot.config.Constants;
import edu.wpi.first.math.util.Units;

public final class OuttakePivotIOSim implements OuttakePivotIO {
    private double currentAngle;
    private double targetAngle;

    public OuttakePivotIOSim() {
        currentAngle = Units.degreesToRotations(Constants.kOuttakePivotInAngle.get());
        targetAngle = currentAngle;
    }

    @Override
    public void updateInputs(Inputs inputs) {
        currentAngle = MathUtil.lerp(currentAngle, targetAngle, 0.1);
        inputs.currentAngleRot = currentAngle;
    }

    @Override
    public void setTargetAngle(double targetAngleRot) {
        targetAngle = targetAngleRot;
    }

    @Override
    public void calibrateEncoder() {
        // no encoder
        currentAngle = 0;
    }
}
