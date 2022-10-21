package com.example.voicejogger;

import android.Manifest;
import android.app.Activity;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.media.AudioFormat;
import android.media.AudioRecord;
import android.media.MediaRecorder;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.util.Log;
import android.widget.CompoundButton;
import android.widget.TextView;
import android.widget.ToggleButton;

import androidx.annotation.NonNull;
import androidx.core.app.ActivityCompat;

import com.example.voicejogger.databinding.ActivityMainBinding;
import com.google.android.gms.wearable.DataClient;
import com.google.android.gms.wearable.DataEvent;
import com.google.android.gms.wearable.DataEventBuffer;
import com.google.android.gms.wearable.DataItem;
import com.google.android.gms.wearable.DataMap;
import com.google.android.gms.wearable.DataMapItem;
import com.google.android.gms.wearable.Wearable;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.UnknownHostException;

public class MainActivity extends Activity implements DataClient.OnDataChangedListener {

    private int port = 50005;
    private String ip_addr = "192.168.1.193";

    private boolean status = false;

    AudioRecord recorder;

    private int sampleRate = 16000 ; // 44100 for music
    private final int channelConfig = AudioFormat.CHANNEL_IN_MONO;
    private final int audioFormat = AudioFormat.ENCODING_PCM_16BIT;
    int minBufSize;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        ActivityMainBinding binding = ActivityMainBinding.inflate(getLayoutInflater());
        setContentView(binding.getRoot());

        ToggleButton toggleButton = binding.togglebuttonStartstop;
        toggleButton.setOnCheckedChangeListener(toggleListener);
    }

    private final CompoundButton.OnCheckedChangeListener toggleListener = (compoundButton, isChecked) -> {
        if(isChecked)
        {
            Log.d("onCheckedChanged","Check ON");
            checkPermission();
            startTransmission();
        }
        else
        {
            Log.d("onCheckedChanged","Check OFF");
            stopTransmission();
        }
    };

    @Override
    protected void onResume() {
        super.onResume();
        Wearable.getDataClient(this).addListener(this);

        // Get data from settings activity
        SharedPreferences preferences = PreferenceManager.getDefaultSharedPreferences(this);
        String get_ip = preferences.getString("ip", "");
        int get_port = preferences.getInt("port", -1);
        int get_rate = preferences.getInt("rate", -1);

        if (get_port != -1)
        {
            port = get_port;
        }

        if (get_rate != -1)
        {
            sampleRate = get_rate;
        }

        if(!get_ip.equals(""))
        {
            ip_addr = get_ip;
        }

        Log.d("onResume", "Server Port set to: " + port);
        Log.d("onResume", "Server IP set to: " + ip_addr);
        Log.d("onResume", "Audio sampling rate set to: " + sampleRate);
    }

    public void checkPermission() {
        if (checkSelfPermission(Manifest.permission.RECORD_AUDIO) != PackageManager.PERMISSION_GRANTED)
        {
            ActivityCompat.requestPermissions(this, new String[]{
                    Manifest.permission.RECORD_AUDIO,}, 1);
        }
    }

    private void startTransmission() {
        if (!status) {
            status = true;
            startStreaming();
        }
    }

    private void stopTransmission() {
        status = false;

        if (recorder != null){
            recorder.release();
            Log.d("stopTransmission","Recorder released");
        }
    }

    public void startStreaming() {
        minBufSize = AudioRecord.getMinBufferSize(sampleRate, channelConfig, audioFormat);
        Thread streamThread = new Thread(() -> {
            try {

                DatagramSocket socket = new DatagramSocket();
                Log.d("startStreaming", "Socket Created");

                byte[] buffer = new byte[minBufSize];

                Log.d("startStreaming","Buffer created of size " + minBufSize);
                DatagramPacket packet;

                final InetAddress destination = InetAddress.getByName(ip_addr);
                Log.d("startStreaming", "Address retrieved");

                checkPermission();
                recorder = new AudioRecord(MediaRecorder.AudioSource.MIC,sampleRate,channelConfig,audioFormat,minBufSize*10);
                Log.d("startStreaming", "Recorder initialized");

                recorder.startRecording();

                while(status) {
                    //reading data from MIC into buffer
                    minBufSize = recorder.read(buffer, 0, buffer.length);

                    //putting buffer in the packet
                    packet = new DatagramPacket (buffer,buffer.length,destination,port);

                    socket.send(packet);
                    System.out.println("MinBufferSize: " +minBufSize);
                }
            } catch(UnknownHostException e) {
                Log.e("startStreaming", "UnknownHostException");
            } catch (IOException e) {
                e.printStackTrace();
                Log.e("startStreaming", "IOException");
            }
        });
        streamThread.start();
    }

    @Override
    public void onDataChanged(@NonNull DataEventBuffer dataEventBuffer) {
        for (DataEvent event : dataEventBuffer) {
            if (event.getType() == DataEvent.TYPE_CHANGED) {
                // DataItem changed
                DataItem item = event.getDataItem();
                if (item.getUri().getPath().compareTo("/voice_jogger_pref") == 0) {

                    SharedPreferences preferences = PreferenceManager.getDefaultSharedPreferences(this);
                    SharedPreferences.Editor editor = preferences.edit();

                    DataMap dataMap = DataMapItem.fromDataItem(item).getDataMap();
                    port = dataMap.getInt("port");
                    sampleRate = dataMap.getInt("rate");
                    ip_addr = dataMap.getString("ip");

                    Log.d("onDataChanged", "Server Port set to: " + port);
                    Log.d("onDataChanged", "Server IP set to: " + ip_addr);
                    Log.d("onDataChanged", "Audio sampling rate set to: " + sampleRate);

                    editor.putInt("port",port);
                    editor.putInt("rate",sampleRate);
                    editor.putString("ip", ip_addr);
                    editor.apply();
                }
            }
        }
    }
}