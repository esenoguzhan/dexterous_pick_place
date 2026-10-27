# ROS 2 between the Jetson app and the robot PC over the USB-C link

Measured on 27 October 2026 with the `restore-perception` app on `wendyos-hach-device-2`
and this laptop as the robot PC (169.254.107.117 on the USB link):

- `wendy` works because it is one TCP connection from the laptop to the agent (port 50051).
- WendyOS drops every other inbound packet on the USB link and rejects the app's outbound
  connections to the laptop, so the UDP traffic DDS uses by default never gets through in
  either direction. The app's current `perception_dds.xml` (UDPv4 on `usb0`, initial peer
  169.254.107.117) can therefore never receive the robot's `/joint_states` and TF.
- A TCP port declared in the app's `wendy.json` **is** reachable from the laptop over the
  USB link (the test bench's port 8090 answered), but only while that app runs.

So the link has to be one TCP connection, opened by the laptop, into a port the perception
app declares. Fast DDS supports exactly that with its TCPv4 transport: the Jetson participant
is the TCP server, the laptop participant the client, and discovery and data both flow over
the single connection.

## Jetson app (restore-perception)

1. Declare the DDS port next to the existing entitlements in `wendy.json`:

   ```json
   { "type": "network", "mode": "host" },
   { "type": "http", "port": 7410 }
   ```

   The `http` entitlement is what opens the port on the USB link; the protocol on it does
   not matter.

2. Replace `perception_dds.xml` with [`jetson_tcp_server.xml`](jetson_tcp_server.xml) and keep
   `FASTRTPS_DEFAULT_PROFILES_FILE` pointing at it for the launch. Rebuild and redeploy with
   `wendy run`.

3. Only one process can own the TCP port, so `ros2` commands run inside the container (service
   calls, `topic echo`) need [`jetson_cli.xml`](jetson_cli.xml) instead:

   ```bash
   FASTRTPS_DEFAULT_PROFILES_FILE=/perception_ws/jetson_cli.xml ros2 service call /perception/capture_scene ...
   ```

## Robot PC (the laptop's robot container)

Copy [`robot_pc_tcp_client.xml`](robot_pc_tcp_client.xml) into the container and start the
robot launch with:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/robot_pc_tcp_client.xml
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch lara5_controller_manager controller_manager.launch.py deployment_mode:=robot
```

Every node in that launch then talks UDP on the laptop as before and additionally keeps one
TCP connection to the Jetson. Check from the Jetson side with `ros2 topic hz /joint_states`
inside the app container; from the laptop with `ros2 node list`, which should show
`/scene_perception_node`.

Both machines must use the same `ROS_DOMAIN_ID` (42).

## Verified

Checked on 27 October 2026 with Fast DDS 2.6 (Humble) in two containers on separate networks,
standing in for the Jetson and the robot PC, with these exact files (addresses substituted):
data flows robot PC to Jetson and Jetson to robot PC over the single TCP connection, a TCP-only
client (the CLI profile) reaches the server, and the robot PC's own nodes still find each other
over UDP. Two things do not work and are avoided here: a UDP transport inside the Jetson's
server participant, and a robot-PC participant without the explicit local UDP peer.
