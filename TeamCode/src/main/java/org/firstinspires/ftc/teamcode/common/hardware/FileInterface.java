package org.firstinspires.ftc.teamcode.common.hardware;

import android.os.Environment;

import com.google.gson.Gson;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.google.gson.stream.JsonReader;
import com.google.gson.stream.JsonWriter;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Reader;
import java.nio.file.Files;
import java.nio.file.Paths;

public class FileInterface {

    public static final String IMU = "imu";
    public static final String INTAKE = "intake";
    public static final String LIFT = "lift";

    public static String read(String key) {
        try {
            JsonReader jsonReader = new JsonReader(new FileReader(Environment.getExternalStorageDirectory().getPath() + "/FIRST/data.json"));
            jsonReader.beginObject();
            while (jsonReader.hasNext()) {
                if (jsonReader.nextName().equals(key)) {
                    return jsonReader.nextString();
                }
            }
            jsonReader.endObject();
        } catch (Exception e) {
            System.out.println("reeeee");
        }
        return "KEY/VALUE_NOT_FOUND";
    }

    public static String write(String key, String value) {
        try {
            JsonWriter jsonWriter = new JsonWriter(new FileWriter(Environment.getExternalStorageDirectory().getPath() + "/FIRST/data.json"));
            jsonWriter.beginObject();
            jsonWriter.name(key).value(value);
        } catch (Exception e) {
            System.out.println("reeeeee2");
        }
        return "KEY/VALUE_NOT_FOUND";
    }
}

class Data {
    float imu;
    int intake;
    int lift;
}


