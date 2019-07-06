package org.example.ahmad.android_test;


import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import geometry_msgs.Pose2D;

public class Sender extends AbstractNodeMain {
    private String topic_name;
    private String goalMessage;
    private Publisher<std_msgs.String> publisher;
    private Publisher<geometry_msgs.Pose2D> publisher1;

    public Sender() {
        this.topic_name = "destination_android";
    }

    public Sender(String topic) {
        this.topic_name = topic;
    }

    public GraphName getDefaultNodeName() {
        return GraphName.of("android_test/sender");
    }

/*
    public void onStart(ConnectedNode connectedNode) {
        final Publisher<std_msgs.String> publisher = connectedNode.newPublisher(this.topic_name, "std_msgs/String");
        connectedNode.executeCancellableLoop(new CancellableLoop() {
            private int sequenceNumber;

            protected void setup() {
                this.sequenceNumber = 0;
            }

            protected void loop() throws InterruptedException {
                std_msgs.String str =  publisher.newMessage();
                str.setData("Testing Android publisher " + this.sequenceNumber);
                publisher.publish(str);
                ++this.sequenceNumber;
                Thread.sleep(1000L);
            }
        });
    }
*/

    public void onStart(ConnectedNode connectedNode) {
        //publisher = connectedNode.newPublisher(this.topic_name, "std_msgs/String");
        publisher1 = connectedNode.newPublisher(this.topic_name, Pose2D._TYPE);
    }

    public void publishMessage(double x,double y, double theta){
        //std_msgs.String str =  publisher.newMessage();

        geometry_msgs.Pose2D pose = publisher1.newMessage();

        pose.setX(x);
        pose.setY(y);
        pose.setTheta(theta);

        publisher1.publish(pose);

        //str.setData(goalMessage);
        //publisher.publish(str);
    }

    public void createGoalString(String x, String y){
        goalMessage = ("x: " + x + ", y: " + y );
    }

}
