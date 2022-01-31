package com.example.voicejogger;

import androidx.appcompat.app.AppCompatActivity;

import android.content.SharedPreferences;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.util.Log;
import android.util.Patterns;
import android.widget.EditText;

import java.util.Arrays;
import java.util.Locale;

public class SettingsActivity extends AppCompatActivity {

    EditText edittext_port;
    EditText edittext_rate;
    EditText edittext_ip;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_settings);

        edittext_port = (EditText) findViewById(R.id.edittext_port);

        edittext_rate = (EditText) findViewById(R.id.edittext_rate);

        edittext_ip = (EditText) findViewById(R.id.edittext_ip);


    }

    protected void onResume() {
        super.onResume();

        SharedPreferences preferences = PreferenceManager.getDefaultSharedPreferences(this);

        Log.d("Settings","Resuming setting activity");

        // Reassign previously saved values

        int get_port = preferences.getInt("port", -1);
        // Only assign if port and rate exist in shared preferences
        if (get_port != -1)
        {
            edittext_port.setText(String.format(Locale.getDefault(), "%d", get_port));
        }

        int get_rate = preferences.getInt("rate", -1);
        if(get_rate != -1)
        {
            edittext_rate.setText(String.format(Locale.getDefault(), "%d", get_rate));
        }

        String get_ip = preferences.getString("ip", "");
        if(!get_ip.equals(""))
        {
            edittext_ip.setText(get_ip);
        }
    }

    @Override
    public void onBackPressed() {
        EditText edittext_port = (EditText) findViewById(R.id.edittext_port);
        EditText edittext_ip = (EditText) findViewById(R.id.edittext_ip);
        EditText edittext_rate = (EditText) findViewById(R.id.edittext_rate);
        String port = edittext_port.getText().toString();
        String ip = edittext_ip.getText().toString();
        String rate = edittext_rate.getText().toString();

        SharedPreferences preferences = PreferenceManager.getDefaultSharedPreferences(this);
        SharedPreferences.Editor editor = preferences.edit();


        // Only put valid data values in Shared Preference
        try {
            int to_set_port = Integer.parseInt(port);
            if(to_set_port>49152 && to_set_port<=65534)
                editor.putInt("port",to_set_port);
            int to_set_rate = Integer.parseInt(rate);

            if(Arrays.asList(new Integer[]{8000, 16000}).contains(to_set_rate))
                editor.putInt("rate",Integer.parseInt(rate));

        } catch (NumberFormatException e) {
            Log.d("Settings", "Invalid port/rate number specified");
        }

        if(Patterns.IP_ADDRESS.matcher(ip).matches())
        {
            editor.putString("ip", ip);
        }

        editor.apply();

        // return back to Main
        finish();

    }
}