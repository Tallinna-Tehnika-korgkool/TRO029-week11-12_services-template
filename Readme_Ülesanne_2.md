# Week 11–12: Kohandatud liidesed (custom `.msg` ja `.srv`) – Ülesanne 2/2

Selles ülesandes lood **enda ROS 2 liidese definitsioonid** (sõnumid ja teenused) eraldi paketis ning kasutad neid **kahes varasemas paketis**:
- `py_pubsub` (publisher/subscriber)
- `py_srvcli` (service/client)

> Mõlemad paketid peavad asuma **samas workspace’is** (nt `~/ros2_ws/src`).

---

## Õpiväljundid
- lood `ament_cmake` interface-paketi (`tutorial_interfaces`)
- lisad oma `.msg` ja `.srv` definitsioonid
- seadistad `CMakeLists.txt` ja `package.xml` nii, et liidesed genereeritakse
- kontrollid, et liidesed on avastatavad `ros2 interface show` abil
- uuendad `py_pubsub` ja `py_srvcli` pakette, et kasutada sinu uusi liideseid

---

## Eeldused
- ROS 2 Humble keskkond töötab (Docker/Devcontainer).
- Sul on workspace, kus on varasemad paketid (nt `py_pubsub`, `py_srvcli`).

---

# A. Loo uus interface-pakett `tutorial_interfaces` (kohustuslik)

1) Ava uus terminal ja source’i ROS 2:
```bash
source /opt/ros/humble/setup.bash
```

2) Mine workspace’i `src` kausta:
```bash
cd ~/ros2_ws/src
```

3) Loo uus pakett (**see peab olema `ament_cmake`**):
```bash
ros2 pkg create --build-type ament_cmake --license Apache-2.0 tutorial_interfaces
```

4) Loo `msg/` ja `srv/` kaustad paketi sees:
```bash
cd ~/ros2_ws/src/tutorial_interfaces
mkdir msg srv
```

---

# B. Loo kohandatud definitsioonid (kohustuslik)

## B1. `msg` definitsioonid

Loo fail `tutorial_interfaces/msg/Num.msg` sisuga:
```text
int64 num
```

Loo fail `tutorial_interfaces/msg/Sphere.msg` sisuga:
```text
geometry_msgs/Point center
float64 radius
```

## B2. `srv` definitsioon

Loo fail `tutorial_interfaces/srv/AddThreeInts.srv` sisuga:
```text
int64 a
int64 b
int64 c
---
int64 sum
```

---

# C. Muuda `CMakeLists.txt` (kohustuslik)

Ava:
```
~/ros2_ws/src/tutorial_interfaces/CMakeLists.txt
```

Lisa (või veendu, et olemas) järgmised read:

```cmake
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "msg/Sphere.msg"
  "srv/AddThreeInts.srv"
  DEPENDENCIES geometry_msgs
)
```

> NB! `rosidl_generate_interfaces` esimene argument peab algama paketi nimega (nt `${PROJECT_NAME}`).

---

# D. Muuda `package.xml` (kohustuslik)

Ava:
```
~/ros2_ws/src/tutorial_interfaces/package.xml
```

Lisa `<package>` elemendi sisse:

```xml
<depend>geometry_msgs</depend>
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

---

# E. Ehita `tutorial_interfaces` (kohustuslik)

Workspace juurest:

```bash
cd ~/ros2_ws
colcon build --packages-select tutorial_interfaces
```

---

# F. Kontrolli, et liidesed on loodud (kohustuslik)

1) Ava uus terminal, mine workspace’i ja source’i:
```bash
cd ~/ros2_ws
source install/setup.bash
```

2) Kontrolli liideseid:

```bash
ros2 interface show tutorial_interfaces/msg/Num
ros2 interface show tutorial_interfaces/msg/Sphere
ros2 interface show tutorial_interfaces/srv/AddThreeInts
```

---

# G. Kasuta uusi liideseid teistes pakettides (hindeline)

## G1. Uuenda `py_pubsub` kasutama `Num.msg`

### Publisher muudatused (`publisher_member_function.py`)
- asenda `std_msgs.msg.String` -> `tutorial_interfaces.msg.Num`
- publisher tüüp `String` -> `Num`
- `msg.data` -> `msg.num`
- logi kuvamine vastavalt numbrile

Näide (muudatustega):
```python
from tutorial_interfaces.msg import Num

self.publisher_ = self.create_publisher(Num, 'topic', 10)

msg = Num()
msg.num = self.i
self.get_logger().info('Publishing: "%d"' % msg.num)
```

### Subscriber muudatused (`subscriber_member_function.py`)
- `String` -> `Num`
- `msg.data` -> `msg.num`

Näide:
```python
from tutorial_interfaces.msg import Num

self.subscription = self.create_subscription(Num, 'topic', self.listener_callback, 10)

def listener_callback(self, msg):
    self.get_logger().info('I heard: "%d"' % msg.num)
```

### `py_pubsub/package.xml`
Lisa:
```xml
<exec_depend>tutorial_interfaces</exec_depend>
```

### Ehita uuesti
```bash
cd ~/ros2_ws
colcon build --packages-select py_pubsub
source install/setup.bash
```

### Käivita (2 terminali)
```bash
ros2 run py_pubsub talker
ros2 run py_pubsub listener
```

---

## G2. Uuenda `py_srvcli` kasutama `AddThreeInts.srv`

### Service muudatused (`service_member_function.py`)
- `from example_interfaces.srv import AddTwoInts` -> `from tutorial_interfaces.srv import AddThreeInts`
- teenuse nimi `add_two_ints` -> `add_three_ints`
- summa `a+b+c`
- logi peab sisaldama a,b,c

### Client muudatused (`client_member_function.py`)
- `AddTwoInts` -> `AddThreeInts`
- teenuse nimi `add_two_ints` -> `add_three_ints`
- loe 3 argumenti (nt `client 2 3 1`)

### `py_srvcli/package.xml`
Lisa:
```xml
<exec_depend>tutorial_interfaces</exec_depend>
```

### Ehita uuesti
```bash
cd ~/ros2_ws
colcon build --packages-select py_srvcli
source install/setup.bash
```

### Käivita (2 terminali)
**Terminal 1:**
```bash
ros2 run py_srvcli service
```

**Terminal 2:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run py_srvcli client 2 3 1
```

---

# Esitamine (kohustuslik)

Sinu GitHub repo peab sisaldama:
- `ros2_ws/src/tutorial_interfaces/` (msg/, srv/, CMakeLists.txt, package.xml)
- muudetud `py_pubsub` failid + `package.xml` sõltuvus
- muudetud `py_srvcli` failid + `package.xml` sõltuvus
- allpool täidetud “Tulemused” sektsioon

Git:
```bash
git status
git add -A
git commit -m "Week 11-12: custom interfaces + updates to pubsub and srvcli"
git push
```

