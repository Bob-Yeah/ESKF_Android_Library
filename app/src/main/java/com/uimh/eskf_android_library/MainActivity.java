package com.uimh.eskf_android_library;

import androidx.appcompat.app.AppCompatActivity;

import android.os.Bundle;
import android.widget.TextView;

import com.uimh.eskf_android_library.databinding.ActivityMainBinding;

public class MainActivity extends AppCompatActivity {

    // Used to load the 'eskf_android_library' library on application startup.
    static {
        System.loadLibrary("eskf_android_library");
    }

    private ActivityMainBinding binding;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        binding = ActivityMainBinding.inflate(getLayoutInflater());
        setContentView(binding.getRoot());

        // Example of a call to a native method
        TextView tv = binding.sampleText;
        tv.setText(stringFromJNI());
    }

    /**
     * A native method that is implemented by the 'eskf_android_library' native library,
     * which is packaged with this application.
     */
    public native String stringFromJNI();
}