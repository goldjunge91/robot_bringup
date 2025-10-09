# robot_bringup Architecture

## Overview

This package orchestrates the launch of all robot components, including hardware interfaces, controllers, micro-ROS agent, and sensor nodes.

## Launch Architecture

```mermaid
graph TB
    subgraph "Main Launch File"
        BL[bringup.launch.py]
    end
    
    subgraph "Hardware Layer"
        MRA[microros_agent.launch.py<br/>Topic Remapping]
        HW[hardware_bringup.launch.py<br/>ros2_control]
    end
    
    subgraph "Controller Layer"
        CL[controller.launch.py<br/>Controller Manager]
        MC[Mecanum Drive Controller]
        JSB[Joint State Broadcaster]
        IB[IMU Broadcaster]
    end
    
    subgraph "Description Layer"
        RSP[Robot State Publisher<br/>URDF/TF]
    end
    
    subgraph "Firmware"
        PICO[Raspberry Pi Pico<br/>micro-ROS Client]
    end
    
    BL --> MRA
    BL --> HW
    BL --> CL
    BL --> RSP
    
    MRA <-->|USB Serial| PICO
    HW --> PICO
    CL --> HW
    
    MC -.->|part of| CL
    JSB -.->|part of| CL
    IB -.->|part of| CL
    
    style BL fill:#4CAF50
    style MRA fill:#2196F3
    style HW fill:#FF9800
    style CL fill:#9C27B0
```

## Launch File Hierarchy

```mermaid
graph LR
    subgraph "Entry Points"
        B[bringup.launch.py]
        HB[hardware_bringup.launch.py]
    end
    
    subgraph "Component Launches"
        MRA[microros_agent.launch.py]
        CL[controller.launch.py]
        RSP[robot_state_publisher]
    end
    
    subgraph "External Packages"
        RD[robot_description]
        RC[robot_controller]
        RHI[robot_hardware_interfaces]
    end
    
    B --> MRA
    B --> HB
    B --> RSP
    
    HB --> CL
    HB --> RHI
    
    CL --> RC
    RSP --> RD
    
    style B fill:#4CAF50
    style HB fill:#FF9800
```

## Sequence Diagram: System Startup

```mermaid
sequenceDiagram
    participant User
    participant BL as bringup.launch.py
    participant MRA as micro-ROS Agent
    participant FW as Pico Firmware
    participant CM as Controller Manager
    participant HWI as Hardware Interface
    participant RSP as Robot State Publisher
    
    User->>BL: ros2 launch robot_bringup bringup.launch.py
    
    par Parallel Launch
        BL->>MRA: Start micro-ROS agent
        MRA->>FW: Connect via USB serial
        FW-->>MRA: Connection established
        
        BL->>RSP: Start robot_state_publisher
        RSP->>RSP: Load URDF
        RSP->>RSP: Publish static transforms
        
        BL->>CM: Start controller_manager
        CM->>HWI: Load hardware interface plugin
        HWI->>HWI: on_init()
        CM->>HWI: on_configure()
        CM->>HWI: on_activate()
        HWI->>FW: Wait for /joint_states
        FW-->>HWI: First joint state received
        HWI-->>CM: Activation successful
    end
    
    CM->>CM: Load controllers
    CM->>CM: Configure controllers
    CM->>CM: Activate controllers
    
    Note over User,RSP: System Ready
    
    User->>User: Verify with ros2 topic list
```

## Data Flow Diagram

```mermaid
flowchart TB
    subgraph "User Input"
        KB[Keyboard Teleop]
        JOY[Joystick]
        NAV[Nav2]
    end
    
    subgraph "Launch Configuration"
        BL[bringup.launch.py]
        PARAMS[Parameters<br/>robot_model<br/>drive_type<br/>microros<br/>serial_port]
    end
    
    subgraph "ROS2 Middleware"
        CV[/cmd_vel]
        JS[/joint_states]
        IMU[/imu/data_raw]
        ODOM[/odom]
    end
    
    subgraph "Hardware Layer"
        MRA[micro-ROS Agent<br/>Topic Remapping]
        HWI[Hardware Interface<br/>ros2_control]
    end
    
    subgraph "Firmware"
        PICO[Pico Firmware<br/>micro-ROS]
    end
    
    KB --> CV
    JOY --> CV
    NAV --> CV
    
    PARAMS --> BL
    BL --> MRA
    BL --> HWI
    
    CV --> HWI
    HWI --> MRA
    MRA <--> PICO
    
    PICO --> MRA
    MRA --> JS
    MRA --> IMU
    MRA --> ODOM
    
    JS --> HWI
    IMU --> HWI
```

## Component Interaction Diagram

```mermaid
graph TB
    subgraph "bringup.launch.py"
        direction LR
        ARGS[Launch Arguments<br/>robot_model<br/>drive_type<br/>microros<br/>serial_port<br/>use_sim_time]
        
        COND{Conditions}
        
        ARGS --> COND
    end
    
    subgraph "Conditional Launches"
        MRA[microros_agent.launch.py<br/>if microros==true]
        HW[hardware_bringup.launch.py<br/>if not simulation]
        SIM[simulation.launch.py<br/>if simulation]
    end
    
    subgraph "Always Launched"
        RSP[robot_state_publisher<br/>URDF + TF]
        RVIZ[RViz2<br/>if use_rviz==true]
    end
    
    COND -->|microros| MRA
    COND -->|hardware| HW
    COND -->|simulation| SIM
    COND --> RSP
    COND -->|use_rviz| RVIZ
    
    style ARGS fill:#4CAF50
    style COND fill:#FF9800
```

## Launch Parameters

```mermaid
graph LR
    subgraph "Launch Arguments"
        RM[robot_model<br/>default: robot_xl]
        DT[drive_type<br/>default: mecanum]
        MR[microros<br/>default: true]
        SP[serial_port<br/>default: /dev/ttyACM0]
        SB[serial_baudrate<br/>default: 115200]
        UST[use_sim_time<br/>default: false]
        UR[use_rviz<br/>default: false]
    end
    
    subgraph "Derived Paths"
        URDF[URDF Path<br/>robot_description/urdf/robot.urdf.xacro]
        CTRL[Controller Config<br/>robot_controller/config/{model}/{drive}.yaml]
    end
    
    subgraph "Node Parameters"
        RSP_P[robot_state_publisher<br/>robot_description<br/>use_sim_time]
        CM_P[controller_manager<br/>controller_config<br/>use_sim_time]
        MRA_P[micro_ros_agent<br/>serial_port<br/>baudrate<br/>remappings]
    end
    
    RM --> URDF
    RM --> CTRL
    DT --> CTRL
    
    URDF --> RSP_P
    CTRL --> CM_P
    UST --> RSP_P
    UST --> CM_P
    
    MR --> MRA_P
    SP --> MRA_P
    SB --> MRA_P
```

## State Machine: Launch Process

```mermaid
stateDiagram-v2
    [*] --> ParseArgs: Launch Started
    
    ParseArgs --> ValidateArgs: Parse launch arguments
    ValidateArgs --> LoadURDF: Validate robot_model, drive_type
    
    LoadURDF --> StartMicroROS: Load URDF from robot_description
    
    state microros_check <<choice>>
    StartMicroROS --> microros_check: Check microros parameter
    microros_check --> LaunchAgent: microros==true
    microros_check --> SkipAgent: microros==false
    
    LaunchAgent --> WaitConnection: Start micro-ROS agent
    WaitConnection --> StartRSP: Wait for firmware connection
    SkipAgent --> StartRSP
    
    StartRSP --> StartControllerManager: Launch robot_state_publisher
    
    StartControllerManager --> LoadHardware: Start controller_manager
    LoadHardware --> ConfigureHardware: Load hardware interface plugin
    ConfigureHardware --> ActivateHardware: Configure hardware
    ActivateHardware --> LoadControllers: Activate hardware (wait for data)
    
    LoadControllers --> ConfigureControllers: Load controller plugins
    ConfigureControllers --> ActivateControllers: Configure controllers
    ActivateControllers --> SystemReady: Activate controllers
    
    SystemReady --> [*]: System operational
    
    note right of SystemReady
        All components running:
        - micro-ROS agent (if enabled)
        - Hardware interface
        - Controllers
        - Robot state publisher
    end note
```

## Topic Remapping Strategy

```mermaid
graph TB
    subgraph "Firmware Topics (with /rt/ prefix)"
        FJS[/rt/joint_states]
        FIMU[/rt/imu/data_raw]
        FODOM[/rt/odom]
        FSENS[/rt/sensors/*]
        FCMD[/rt/cmd_vel]
    end
    
    subgraph "micro-ROS Agent Remapping"
        direction TB
        R1[/rt/joint_states → /joint_states]
        R2[/rt/imu/data_raw → /imu/data_raw]
        R3[/rt/odom → /odom]
        R4[/rt/sensors/* → /sensors/*]
        R5[/cmd_vel → /rt/cmd_vel]
    end
    
    subgraph "Standard ROS2 Topics"
        JS[/joint_states]
        IMU[/imu/data_raw]
        ODOM[/odom]
        SENS[/sensors/*]
        CMD[/cmd_vel]
    end
    
    FJS --> R1 --> JS
    FIMU --> R2 --> IMU
    FODOM --> R3 --> ODOM
    FSENS --> R4 --> SENS
    CMD --> R5 --> FCMD
    
    style R1 fill:#2196F3
    style R2 fill:#2196F3
    style R3 fill:#2196F3
    style R4 fill:#2196F3
    style R5 fill:#2196F3
```

## Configuration Files

### bringup.launch.py

**Purpose**: Main entry point for launching the complete robot system

**Key Features**:
- Conditional launching based on parameters
- URDF loading and robot_state_publisher
- micro-ROS agent with topic remapping
- Hardware interface and controller manager
- RViz visualization (optional)

**Launch Arguments**:
| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `robot_model` | string | robot_xl | Robot model name |
| `drive_type` | string | mecanum | Drive type (mecanum/diff) |
| `microros` | bool | true | Enable micro-ROS agent |
| `serial_port` | string | /dev/ttyACM0 | Pico serial port |
| `serial_baudrate` | int | 115200 | Serial baud rate |
| `use_sim_time` | bool | false | Use simulation time |
| `use_rviz` | bool | false | Launch RViz |

### microros_agent.launch.py

**Purpose**: Launch micro-ROS agent with topic remapping

**Key Features**:
- Auto-detect Pico serial device
- Topic remapping (/rt/* ↔ /*)
- Configurable serial parameters
- Verbose logging option

**Topic Remappings**:
```python
remappings = [
    ('/rt/joint_states', '/joint_states'),
    ('/rt/imu/data_raw', '/imu/data_raw'),
    ('/rt/odom', '/odom'),
    ('/rt/sensors/range_tof', '/sensors/range_tof'),
    ('/rt/sensors/range_ultrasonic', '/sensors/range_ultrasonic'),
    ('/rt/sensors/illuminance', '/sensors/illuminance'),
    ('/cmd_vel', '/rt/cmd_vel'),
]
```

### hardware_bringup.launch.py

**Purpose**: Launch hardware interface and controllers

**Key Features**:
- Load ros2_control configuration
- Start controller manager
- Include controller.launch.py
- Hardware-specific parameters

## Error Handling

```mermaid
flowchart TD
    START[Launch Started] --> CHECK_ARGS{Valid Arguments?}
    
    CHECK_ARGS -->|No| ERROR1[Error: Invalid robot_model or drive_type]
    CHECK_ARGS -->|Yes| LOAD_URDF{URDF Exists?}
    
    LOAD_URDF -->|No| ERROR2[Error: URDF file not found]
    LOAD_URDF -->|Yes| START_AGENT{microros enabled?}
    
    START_AGENT -->|Yes| AGENT_START{Agent Started?}
    START_AGENT -->|No| START_HW
    
    AGENT_START -->|No| ERROR3[Error: Failed to start agent]
    AGENT_START -->|Yes| WAIT_FW{Firmware Connected?}
    
    WAIT_FW -->|Timeout| WARN1[Warning: No firmware connection<br/>Continue anyway]
    WAIT_FW -->|Yes| START_HW[Start Hardware Interface]
    WARN1 --> START_HW
    
    START_HW --> HW_INIT{Hardware Initialized?}
    
    HW_INIT -->|No| ERROR4[Error: Hardware init failed]
    HW_INIT -->|Yes| HW_ACTIVATE{Hardware Activated?}
    
    HW_ACTIVATE -->|No| ERROR5[Error: No data from firmware]
    HW_ACTIVATE -->|Yes| LOAD_CTRL[Load Controllers]
    
    LOAD_CTRL --> CTRL_OK{Controllers Loaded?}
    
    CTRL_OK -->|No| ERROR6[Error: Controller load failed]
    CTRL_OK -->|Yes| SUCCESS[System Ready]
    
    ERROR1 --> END[Exit]
    ERROR2 --> END
    ERROR3 --> END
    ERROR4 --> END
    ERROR5 --> END
    ERROR6 --> END
    SUCCESS --> RUNNING[Running]
    
    style ERROR1 fill:#f44336
    style ERROR2 fill:#f44336
    style ERROR3 fill:#f44336
    style ERROR4 fill:#f44336
    style ERROR5 fill:#f44336
    style ERROR6 fill:#f44336
    style WARN1 fill:#FF9800
    style SUCCESS fill:#4CAF50
```

## Dependencies

- **robot_description**: URDF and robot model
- **robot_controller**: Controller configurations
- **robot_hardware_interfaces**: Hardware interface plugin
- **micro_ros_agent**: micro-ROS bridge
- **robot_state_publisher**: TF tree publishing
- **controller_manager**: ros2_control controller management

## Usage Examples

### Basic Launch
```bash
ros2 launch robot_bringup bringup.launch.py
```

### With RViz
```bash
ros2 launch robot_bringup bringup.launch.py use_rviz:=true
```

### Custom Serial Port
```bash
ros2 launch robot_bringup bringup.launch.py serial_port:=/dev/ttyUSB0
```

### Without micro-ROS (Simulation)
```bash
ros2 launch robot_bringup bringup.launch.py microros:=false use_sim_time:=true
```

### Differential Drive
```bash
ros2 launch robot_bringup bringup.launch.py drive_type:=diff
```

## Troubleshooting

| Issue | Possible Cause | Solution |
|-------|---------------|----------|
| Launch fails immediately | Invalid arguments | Check robot_model and drive_type |
| No firmware connection | Pico not connected | Check USB connection, verify /dev/ttyACM0 |
| Hardware activation fails | No data from firmware | Verify firmware is running, check micro-ROS agent |
| Controllers fail to load | Wrong configuration | Check controller config matches drive_type |
| No topics visible | micro-ROS agent not running | Verify agent started, check remappings |
