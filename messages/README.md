# LISUM Messages

## MAVLink

**LISUM** shall use MAVLink message and command IDs ranging from
**58880** - **59519**. Each topic shall be given range of **64** possible
message IDs.

### Core Messages

Core Messages IDs shall be contained in range of **58880** - **58943**. There
are no Core Messages for now.

### Device Configuration Messages

Device Configuration Messages IDs shall be contained in range of
**58944** - **59007**. There are no Device Configuration Messages for now.

### Log Messages

Log Messages IDs shall be contained in range of **59008** - **59071**. There are
no Log Messages for now.

### Power Messages

Power Messages IDs shall be contained in range of **59072** - **59135**. Power
Messages are:

|Name                         |Message ID  |Description                |
|-----------------------------|------------|---------------------------|
|LISUM_POWER_MOTOR_DATA       |59072       |Motor data                 |
|LISUM_POWER_MOTOR_SCALED_DATA|59073       |Motor scaled data          |
|LISUM_POWER_HORNET_ACT_DATA  |59080       |Hornet actuator data       |

### Network and telemetry Messages

Network and telemetry Messages IDs shall be contained in range of
**59136** - **59199**. There are no Network and telemetry Messages for now.

### Sensor Messages

Sensor Messages IDs shall be contained in range of **59200** - **59263**. Sensor
Messages are:

|Name                       |Message ID  |Description                         |
|---------------------------|------------|------------------------------------|
|LISUM_SENSOR_AIRSPEED_DATA |59200       |Airspeed information from a sensor  |

### GNSS Messages

GNSS Messages IDs shall be contained in range of **59264** - **59327**. GNSS
Messages are:

|Name                   |Message ID  |Description        |
|-----------------------|------------|-------------------|
|LISUM_GNSS_RECV_DATA   |59264       |GNSS receiver data |

### Manual control Messages

Manual control Messages IDs shall be contained in range of
**59328** - **59391**. Manual control Messages are:

|Name                       |Message ID  |Description            |
|---------------------------|------------|-----------------------|
|LISUM_MANUAL_CTRL_HORNET   |59328       |Hornet manual control  |

### Autopilot Messages

Autopilot Messages IDs shall be contained in range of **59392** - **59455**.
There are no Autopilot Messages for now.

### Camera Messages

Camera Messages IDs shall be contained in range of **59456** - **59519**. There
are no Camera Messages for now.

## Cyphal

There no LISUM Cyphal messages for now.