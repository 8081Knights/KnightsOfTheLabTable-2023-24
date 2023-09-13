package org.firstinspires.ftc.teamcode.Tests.JSONStuff;

import com.google.gson.Gson;
import com.google.gson.JsonSyntaxException;
import com.google.gson.reflect.TypeToken;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.List;

public class JsonReader {
    private final Gson gsonAgain = new Gson();
    private final String filePathName = "test"; // added a semicolon here
    List<JsonObjects.Movements> instructions = gsonAgain.fromJson(loadJSONFromAsset(filePathName),
            new TypeToken<List<JsonObjects.Movements>>() {}.getType()); // changed this line



    public String loadJSONFromAsset(String filePath) {
        StringBuilder sb = new StringBuilder();
        try {
            File file = new File(filePath);
            try (FileInputStream fis = new FileInputStream(file)) {
                int size = fis.available();
                byte[] buffer = new byte[size];
                fis.read(buffer);
                sb.append(new String(buffer, "UTF-8"));
            }
        } catch (IOException | JsonSyntaxException e) {
            e.printStackTrace();
            return null;
        }
        return sb.toString();
    }
}
