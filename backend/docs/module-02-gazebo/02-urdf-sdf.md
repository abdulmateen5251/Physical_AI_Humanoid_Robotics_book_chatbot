# Chapter 2: URDF to SDF Conversion

## Understanding URDF vs SDF

| Feature | URDF | SDF |
|---------|------|-----|
| **Format** | XML for robots | XML for worlds + robots |
| **Physics** | Basic | Advanced |
| **Sensors** | Limited | Full support |
| **Simulation** | Design oriented | Simulation oriented |
| **Use Case** | ROS description | Gazebo/sim environments |

## Converting URDF to SDF

**Original URDF (2-link arm):**
```xml
<?xml version="1.0"?>
<robot name="simple_arm">
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <link name="link1">
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.05"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.1 0.1 0.3"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.3"/>
      </geometry>
    </collision>
  </link>

  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.57" effort="10" velocity="1.0"/>
  </joint>
</robot>
```

**Converted to SDF:**
```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="simple_arm">
    <link name="base_link">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.1</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.1</iyy>
          <iyz>0</iyz>
          <izz>0.1</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <cylinder>
            <length>0.1</length>
            <radius>0.05</radius>
          </cylinder>
        </geometry>
      </visual>
      <collision name="collision">
        <geometry>
          <cylinder>
            <length>0.1</length>
            <radius>0.05</radius>
          </cylinder>
        </geometry>
      </collision>
    </link>

    <link name="link1">
      <inertial>
        <mass>0.5</mass>
        <inertia>
          <ixx>0.05</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.05</iyy>
          <iyz>0</iyz>
          <izz>0.05</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.1 0.1 0.3</size>
          </box>
        </geometry>
      </visual>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.1 0.1 0.3</size>
          </box>
        </geometry>
      </collision>
    </link>

    <joint name="joint1" type="revolute">
      <parent>base_link</parent>
      <child>link1</child>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>0</lower>
          <upper>1.57</upper>
          <effort>10</effort>
          <velocity>1.0</velocity>
        </limit>
      </axis>
    </joint>
  </model>
</sdf>
```

## Automatic Conversion

Use the `urdf2sdf` tool:

```bash
# Convert URDF to SDF
urdf2sdf.py your_robot.urdf > your_robot.sdf

# Or with gz tool
gz sdf print -o sdf your_robot.urdf > your_robot.sdf
```

## Key Differences to Watch

1. **Inertia format**: URDF uses attributes, SDF uses nested elements
2. **Geometry**: SDF geometry tags have more parameters (pose, size format)
3. **Limits**: In SDF, limits are nested under axis
4. **Namespacing**: SDF uses full namespace hierarchies
5. **Friction**: SDF has more detailed friction parameters

## Adding SDF-Specific Features

Once in SDF, you can add Gazebo-specific plugins:

```xml
<model name="my_robot">
  <!-- ... links and joints ... -->
  
  <plugin filename="libgazebo_ros_tricycle_drive.so" name="tricycle_drive">
    <left_wheel_joint>wheel_l</left_wheel_joint>
    <right_wheel_joint>wheel_r</right_wheel_joint>
    <steering_joint>steer</steering_joint>
    <ros>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
    </ros>
  </plugin>
</model>
```
