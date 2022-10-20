package com.example.voicejogger;

import android.Manifest;
import android.app.Activity;
import android.content.pm.PackageManager;
import android.media.AudioFormat;
import android.media.AudioRecord;
import android.media.MediaRecorder;
import android.os.Bundle;
import android.util.Log;
import android.widget.CompoundButton;
import android.widget.TextView;
import android.widget.ToggleButton;

import androidx.core.app.ActivityCompat;

import com.example.voicejogger.databinding.ActivityMainBinding;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.UnknownHostException;

public class MainActivity extends Activity {

    private int port = 50005;
    private String ip_addr = "192.168.1.193";

    private boolean status = false;

    private TextView mTextView;

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

        mTextView = binding.text;
        ToggleButton toggleButton = binding.togglebuttonStartstop;
        toggleButton.setOnCheckedChangeListener(toggleListener);
    }

    private final CompoundButton.OnCheckedChangeListener toggleListener = (compoundButton, isChecked) -> {
        if(isChecked)
        {
            Log.d("onCheckedChanged","Check ON");
            mTextView.setText("Transmission on");
            checkPermission();
            startTransmission();
        }
        else
        {
            Log.d("onCheckedChanged","Check OFF");
            mTextView.setText("Transmission off");
            stopTransmission();
        }
    };

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
}