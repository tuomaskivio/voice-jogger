package com.example.voicejogger;

import android.app.Activity;
import android.os.Bundle;
import android.widget.TextView;

import com.example.voicejogger.databinding.ActivitySettingsBinding;

public class SettingsActivity extends Activity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        ActivitySettingsBinding binding = ActivitySettingsBinding.inflate(getLayoutInflater());
        setContentView(binding.getRoot());

        TextView textViewIP = binding.textViewIP;
        TextView textViewPort = binding.textViewPort;
        TextView textViewRate = binding.textViewRate;

        Bundle extras = getIntent().getExtras();
        textViewIP.setText(extras.getString("ip"));
        textViewPort.setText(Integer.toString(extras.getInt("port")));
        textViewRate.setText(Integer.toString(extras.getInt("rate")));
    }
}