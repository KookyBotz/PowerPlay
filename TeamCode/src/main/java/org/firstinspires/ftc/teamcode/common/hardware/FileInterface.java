package org.firstinspires.ftc.teamcode.common.hardware;

import android.os.Environment;

import com.google.gson.Gson;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import java.io.BufferedWriter;
import java.io.IOException;
import java.io.Reader;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.Map;

public class FileInterface {

    public static final String IMU = "imu";
    public static final String INTAKE = "intake";
    public static final String LIFT = "lift";

    public static String read(String key) {
        try {
            Reader reader = Files.newBufferedReader(Paths.get(Environment.getExternalStorageDirectory().getPath() + "/FIRST/constants.json"));

            JsonObject parser = new JsonParser().parse(reader).getAsJsonObject();
            return parser.get(key).getAsString();

        } catch (IOException e) {
            e.printStackTrace();
        }

        return null;
    }

    public static void write(String key, String value) {
        try {
            BufferedWriter writer = Files.newBufferedWriter(Paths.get(Environment.getExternalStorageDirectory().getPath() + "/FIRST/constants.json"));

            Map<String, Object> values = new HashMap<>();
            values.put(key, value);

            writer.write(new Gson().toJson(values));
            writer.close();

        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
