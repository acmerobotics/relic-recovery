package com.acmerobotics.velocityvortex.opmodes.tester;

import com.qualcomm.robotcore.hardware.HardwareDevice;

import java.lang.reflect.InvocationTargetException;

public class TesterBinding<T extends HardwareDevice> {

    private String nameRegex;
    private Class<T> deviceClass;
    private Class<? extends Tester<T>> testerClass;

    public TesterBinding(String nameRegex, Class<T> deviceClass, Class<? extends Tester<T>> testerClass) {
        this.nameRegex = nameRegex;
        this.deviceClass = deviceClass;
        this.testerClass = testerClass;
    }

    public boolean matches(String name, HardwareDevice device) {
        return ((nameRegex.length() == 0 || name.matches(nameRegex)) && deviceClass.isInstance(device));
    }

    public Tester<T> newTester(String name, HardwareDevice device) {
        try {
            return (Tester<T>) testerClass.getConstructors()[0].newInstance(name, device);
        } catch (InstantiationException e) {
            e.printStackTrace();
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        } catch (InvocationTargetException e) {
            e.printStackTrace();
        }
        return null;
    }

    public boolean isValid() {
        return testerClass != null;
    }

}
