/**
* @author ahmadhassan.mirza@gmail.com (Ahmad H. Mirza)
*/

package org.example.ahmad.android_test;

import android.content.Intent;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.view.View;

public class MainActivity extends AppCompatActivity {
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);

    }
    /*
    *Function to start the automatic navigation activity.
    *Uses several sensor inputs to drive the robot
    * to the location specified by the user using cartesian co-ordinates.
    */
    public void autoNavStart(View view) {

        Intent intent = new Intent(this, AutoNav.class);
        startActivity(intent);
    }
    /*
    *Function to start the map creation activity.
    *Uses lidar input to create a 2-d map of the surroundings.
    */    
    public void mapViewStart(View view) {

        Intent intent = new Intent(this, CreateMap.class);
        startActivity(intent);
    }

}
