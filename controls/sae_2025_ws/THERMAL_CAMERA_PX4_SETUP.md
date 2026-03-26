# Thermal Camera — PX4-Autopilot Setup

These changes live **outside the monorepo** in your local `PX4-Autopilot` clone.
They are not tracked by git here, so each developer needs to apply them manually.

---

## 1. Update the Gazebo model

The model `x500_thermal_cam_down` already exists in PX4 but was just a copy of
`x500_mono_cam_down`. Replace both files as shown below.

### `Tools/simulation/gz/models/x500_thermal_cam_down/model.config`

Change the `<name>` field (it was incorrectly set to `x500_mono_cam_down`):

```xml
<?xml version="1.0"?>
<model>
  <name>x500_thermal_cam_down</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author>
    <name>Daniel Mesham</name>
    <email>daniel@auterion.com</email>
  </author>
  <description>An X500 with a downward-facing mono camera and thermal camera.</description>
</model>
```

### `Tools/simulation/gz/models/x500_thermal_cam_down/model.sdf`

Replace the entire file:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<sdf version='1.9'>
  <model name='x500_thermal_cam_down'>
    <self_collide>false</self_collide>
    <include merge='true'>
      <uri>x500</uri>
    </include>

    <!-- Regular downward mono camera -->
    <include merge='true'>
      <uri>model://mono_cam</uri>
      <pose>0 0 .10 0 1.5707 0</pose>
      <name>mono_cam</name>
    </include>
    <joint name="CameraJoint" type="fixed">
      <parent>base_link</parent>
      <child>camera_link</child>
      <pose relative_to="base_link">0 0 0 0 1.5707 0</pose>
    </joint>

    <!-- Downward-facing thermal camera -->
    <link name="thermal_camera_link">
      <inertial>
        <mass>0.050</mass>
        <inertia>
          <ixx>0.00004</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.00004</iyy>
          <iyz>0</iyz>
          <izz>0.00004</izz>
        </inertia>
      </inertial>
      <sensor name="thermal_camera" type="thermal_camera">
        <gz_frame_id>thermal_camera_link</gz_frame_id>
        <camera>
          <horizontal_fov>1.047</horizontal_fov>
          <image>
            <width>320</width>
            <height>240</height>
            <format>L16</format>
          </image>
          <clip>
            <near>0.1</near>
            <far>100</far>
          </clip>
        </camera>
        <always_on>1</always_on>
        <update_rate>10</update_rate>
        <visualize>true</visualize>
        <topic>thermal_camera/image</topic>
      </sensor>
    </link>
    <joint name="ThermalCameraJoint" type="fixed">
      <parent>base_link</parent>
      <child>thermal_camera_link</child>
      <pose relative_to="base_link">0 0 0 0 1.5707 0</pose>
    </joint>
  </model>
</sdf>
```

---

## 2. Register the airframe

### Create `ROMFS/px4fmu_common/init.d-posix/airframes/4015_gz_x500_thermal_cam_down`

```sh
#!/bin/sh
#
# @name Gazebo x500 thermal cam
#
# @type Quadrotor
#

PX4_SIM_MODEL=${PX4_SIM_MODEL:=x500_thermal_cam_down}

. ${R}etc/init.d-posix/airframes/4001_gz_x500
```

Make sure it's executable:
```bash
chmod +x ROMFS/px4fmu_common/init.d-posix/airframes/4015_gz_x500_thermal_cam_down
```

### Edit `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`

Add the new airframe on the line after `4014_gz_x500_mono_cam_down`:

```cmake
    4014_gz_x500_mono_cam_down
    4015_gz_x500_thermal_cam_down   # <-- add this line
    4016_gz_x500_lidar_down
```

---

## 3. Rebuild PX4

```bash
cd ~/PX4-Autopilot
make px4_sitl
```

---

## 4. Copy model to Gazebo cache

Gazebo resolves models from `~/.simulation-gazebo/models/`, not directly from the
PX4 source tree. After rebuilding, copy the model there:

```bash
cp -r ~/PX4-Autopilot/Tools/simulation/gz/models/x500_thermal_cam_down \
      ~/.simulation-gazebo/models/
```

You only need to do this once (or after editing the model SDF).

---

## Summary of files changed in PX4-Autopilot

| File | Change |
|------|--------|
| `Tools/simulation/gz/models/x500_thermal_cam_down/model.sdf` | Added thermal camera sensor + joint |
| `Tools/simulation/gz/models/x500_thermal_cam_down/model.config` | Fixed model name |
| `ROMFS/px4fmu_common/init.d-posix/airframes/4015_gz_x500_thermal_cam_down` | New file — airframe definition |
| `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt` | Registered airframe 4015 |
