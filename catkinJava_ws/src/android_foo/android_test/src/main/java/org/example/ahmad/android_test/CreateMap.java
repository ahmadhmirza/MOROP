


/*
 * Copyright (C) 2013 OSRF.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */


/*
 * Copyright (C) 2012 Google Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.example.ahmad.android_test;

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;

import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.widget.Toast;
import android.widget.ToggleButton;

import org.ros.address.InetAddressFactory;
import org.ros.android.BitmapFromCompressedImage;
import org.ros.android.RosActivity;
import org.ros.android.view.RosImageView;
import org.ros.android.view.VirtualJoystickView;
import org.ros.android.view.visualization.VisualizationView;
import org.ros.android.view.visualization.layer.CameraControlLayer;
import org.ros.android.view.visualization.layer.CameraControlListener;
import org.ros.android.view.visualization.layer.LaserScanLayer;
import org.ros.android.view.visualization.layer.Layer;
import org.ros.android.view.visualization.layer.OccupancyGridLayer;
import org.ros.android.view.visualization.layer.PathLayer;
import org.ros.android.view.visualization.layer.PosePublisherLayer;
import org.ros.android.view.visualization.layer.PoseSubscriberLayer;
import org.ros.android.view.visualization.layer.RobotLayer;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.time.NtpTimeProvider;

import java.util.concurrent.TimeUnit;

import sensor_msgs.CompressedImage;

public class CreateMap extends RosActivity {

    private static final String MAP_FRAME = "map";
    private static final String ROBOT_FRAME = "base_link";

    //private final SystemCommands systemCommands;

    private VisualizationView visualizationView;
    private VirtualJoystickView virtualJoystickView;
    private ToggleButton followMeToggleButton;
    private CameraControlLayer cameraControlLayer;
    private RosImageView<CompressedImage> image;

    public CreateMap() {
        super("Map Viewer", "Map Viewer");
        //systemCommands = new SystemCommands();
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN,
                WindowManager.LayoutParams.FLAG_FULLSCREEN);
        setContentView(R.layout.activity_create_map);
        visualizationView = findViewById(R.id.visualization);
        virtualJoystickView = findViewById(R.id.virtual_joystick);
        image = findViewById(R.id.image);
        image.setTopicName("/usb_cam/image_raw/compressed");
        image.setMessageType(sensor_msgs.CompressedImage._TYPE);
        image.setMessageToBitmapCallable(new BitmapFromCompressedImage());


        cameraControlLayer = new CameraControlLayer();
        visualizationView.onCreate(Lists.<Layer>newArrayList(cameraControlLayer,
                new OccupancyGridLayer("map"), new LaserScanLayer("scan"), new RobotLayer(ROBOT_FRAME),
                new PathLayer("move_base/NavfnROS/plan"), new PathLayer(
                        "move_base_dynamic/NavfnROS/plan"), new LaserScanLayer("base_scan"),
                new PoseSubscriberLayer("simple_waypoints_server/goal_pose"), new PosePublisherLayer(
                        "simple_waypoints_server/goal_pose"), new RobotLayer("base_footprint")));
        followMeToggleButton = (ToggleButton) findViewById(R.id.follow_me_toggle_button);
        enableFollowMe();

        virtualJoystickView.setTopicName("cmd_vel");

    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        visualizationView.init(nodeMainExecutor);

        cameraControlLayer.addListener(new CameraControlListener() {
            @Override
            public void onZoom(float focusX, float focusY, float factor) {
                disableFollowMe();
            }

            @Override
            public void onTranslate(float distanceX, float distanceY) {
                disableFollowMe();
            }

            @Override
            public void onRotate(float focusX, float focusY, double deltaAngle) {
                disableFollowMe();
            }

            @Override
            public void onDoubleTap(float x, float y) {
            }
        });

        NodeConfiguration nodeConfiguration =
                NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(),
                        getMasterUri());
/*        NtpTimeProvider ntpTimeProvider =
                new NtpTimeProvider(InetAddressFactory.newFromHostString("192.168.0.1"),
                        nodeMainExecutor.getScheduledExecutorService());
        ntpTimeProvider.startPeriodicUpdates(1, TimeUnit.MINUTES);
        nodeConfiguration.setTimeProvider(ntpTimeProvider);*/


        nodeMainExecutor
                .execute(virtualJoystickView, nodeConfiguration.setNodeName("virtual_joystick"));
        //nodeMainExecutor.execute(systemCommands, nodeConfiguration);

        nodeMainExecutor.execute(visualizationView, nodeConfiguration.setNodeName("android/map_viewww"));
        nodeMainExecutor.execute(image, nodeConfiguration.setNodeName("android/video_view"));
    }

    public void onClearMapButtonClicked(View view) {
        toast("Clearing map...");
        //systemCommands.reset();
        enableFollowMe();
    }

    public void onSaveMapButtonClicked(View view) {
        toast("Saving map...");
       // systemCommands.saveGeotiff();
    }

    private void toast(final String text) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Toast toast = Toast.makeText(CreateMap.this, text, Toast.LENGTH_SHORT);
                toast.show();
            }
        });
    }

    public void onFollowMeToggleButtonClicked(View view) {
        boolean on = ((ToggleButton) view).isChecked();
        if (on) {
            enableFollowMe();
        } else {
            disableFollowMe();
        }
    }

    private void enableFollowMe() {
        Preconditions.checkNotNull(visualizationView);
        Preconditions.checkNotNull(followMeToggleButton);
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                visualizationView.getCamera().jumpToFrame(ROBOT_FRAME);
                followMeToggleButton.setChecked(true);
            }
        });
    }

    private void disableFollowMe() {
        Preconditions.checkNotNull(visualizationView);
        Preconditions.checkNotNull(followMeToggleButton);
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                visualizationView.getCamera().setFrame(MAP_FRAME);
                followMeToggleButton.setChecked(false);
            }
        });
    }

    public void startCamerView(View v){
        Intent intent = new Intent(this, CameraViewActivity.class);
        startActivity(intent);
    }
}
