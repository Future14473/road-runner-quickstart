package org.firstinspires.ftc.teamcode.Bluetooth;

import android.app.Activity;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;

public class BluetoothConvenient {

    // Bluetooth Debugging
    BluetoothClient bluetoothClient;
    // Forth Interpreter
    Interpreter interpreter;


    public BluetoothConvenient(Telemetry telemetry, HardwareMap hardwareMap, Object o){
        // Intialize Bluetooth Stuff
        interpreter = new Interpreter(
                message -> bluetoothClient.send(message),
                o);

        bluetoothClient = new BluetoothClient(
                (Activity) hardwareMap.appContext, "C8:21:58:6A:A3:A0",
                "00001101-0000-1000-8000-00805F9B34FB",
                telemetry,
                response -> interpreter.nexttoks.addAll(
                        Arrays.asList(response.split(" "))));

        bluetoothClient.startHostSession();
        new Thread(() -> interpreter.begin()).start();
    }
}
