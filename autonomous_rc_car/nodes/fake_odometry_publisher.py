self.x = 0.0
self.y = 0.0
self.theta = 0.0
self.last_time = self.get_clock().now()

def publish_fake_odom(self):
    now = self.get_clock().now()
    dt = (now - self.last_time).nanoseconds / 1e9
    self.last_time = now

    v = 0.05  # simulate forward velocity (m/s)
    self.x += v * dt

    # Create odometry message
    odom = Odometry()
    odom.header.stamp = now.to_msg()
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_link"

    odom.pose.pose.position.x = float(self.x)
    odom.pose.pose.orientation = Quaternion(w=1.0)
    odom.twist.twist.linear.x = v

    self.odom_pub.publish(odom)

    # TF broadcast
    tf_msg = TransformStamped()
    tf_msg.header.stamp = now.to_msg()
    tf_msg.header.frame_id = "odom"
    tf_msg.child_frame_id = "base_link"
    tf_msg.transform.translation.x = float(self.x)
    tf_msg.transform.rotation = Quaternion(w=1.0)
    self.tf_broadcaster.sendTransform(tf_msg)

