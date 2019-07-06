/*
 * Copyright (C) 2011 Google Inc.
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

import android.os.Bundle;
import android.view.View;
import android.widget.EditText;

import org.ros.android.MessageCallable;
import org.ros.android.RosActivity;
import org.ros.android.view.RosTextView;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class AutoNav extends RosActivity {

    private RosTextView<std_msgs.String> rosTextView;
    private Sender sender; //publisher object
    private Reciever reciever;
    private EditText xGoalView,yGoalView,thetaGoalView;


    public AutoNav() {
        // The RosActivity constructor configures the notification title
        super("Android ROS Test", "Android ROS Test");
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_auto_nav);

        rosTextView = findViewById(R.id.text);
        rosTextView.setTopicName("android_info_msg");
        rosTextView.setMessageType(std_msgs.String._TYPE);
        rosTextView.setMessageToStringCallable(new MessageCallable<String, std_msgs.String>() {
            @Override
            public String call(std_msgs.String message) {
                return message.getData();
            }
        });

        xGoalView = findViewById(R.id.xInput);
        xGoalView.setText("0");
        yGoalView = findViewById(R.id.yInput);
        yGoalView.setText("0");
        thetaGoalView = findViewById(R.id.thetaInput);
        thetaGoalView.setText("0");
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        sender = new Sender();
        reciever = new Reciever();
        // At this point, the user has already been prompted to either enter the URI
        // of a master to use or to start a master locally.

        // The user can easily use the selected ROS Hostname in the master chooser
        // activity.
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(getRosHostname());
        nodeConfiguration.setMasterUri(getMasterUri());
        nodeMainExecutor.execute(sender, nodeConfiguration);
        // The RosTextView is also a NodeMain that must be executed in order to
        // start displaying incoming messages.
        nodeMainExecutor.execute(rosTextView, nodeConfiguration);
        nodeMainExecutor.execute(reciever, nodeConfiguration);
    }

    public void sendGoal(View view) {
        //String goal = xGoalView.getText().toString() + yGoalView.getText().toString();
        //sender.createGoalString(xGoalView.getText().toString(), yGoalView.getText().toString());
        //sender.publishMessage();
        double x = Double.parseDouble(xGoalView.getText().toString());
        double y = Double.parseDouble(yGoalView.getText().toString());
        double theta = Double.parseDouble(thetaGoalView.getText().toString());

        sender.publishMessage(x,y,theta);
    }
}
