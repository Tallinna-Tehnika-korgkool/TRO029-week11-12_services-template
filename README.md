# Nädal 11–12: Services + kohandatud liidesed

Selle nädala jooksul teed **kaks** ülesannet:
1. Service + Client (`py_srvcli`)
2. Kohandatud liidesed (`tutorial_interfaces`) ja nende kasutamine
   eelnevates pakettides (`py_pubsub`, `py_srvcli`)

## Eeldused
- ROS 2 Humble keskkond töötab (vt nädal 01–02).
- Sul on workspace'is olemas pakett `py_pubsub` (nädal 09–10).

---

# Osa 1: Service + Client (`py_srvcli`)

## Õpiväljundid
- lood ROS 2 Python paketi `ament_python`
- kasutad teenuse tüüpi `example_interfaces/srv/AddTwoInts`
- kirjutad **service node** ja **client node**
- seadistad `entry_points`, et `ros2 run` töötaks

## Ülesanne A: loo pakett (kohustuslik)

```bash
cd /workspace/ros2_ws/src
ros2 pkg create --build-type ament_python --license Apache-2.0 py_srvcli --dependencies rclpy example_interfaces
```

## Ülesanne B: uuenda `package.xml` metaandmed (kohustuslik)

Ava `/workspace/ros2_ws/src/py_srvcli/package.xml` ja täida:
```xml
<description>Python client server tutorial</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
```

## Ülesanne C: uuenda `setup.py` metaandmed (kohustuslik)

Ava `/workspace/ros2_ws/src/py_srvcli/setup.py` ja täida `setup()`
väljad nii, et need klapiksid `package.xml`-iga.

## Ülesanne D: kirjuta service node (kohustuslik)

Mine kausta `/workspace/ros2_ws/src/py_srvcli/py_srvcli` ja loo fail
`service_member_function.py`:

```python
from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Ülesanne E: lisa service entry point (kohustuslik)

Ava `setup.py` ja lisa `entry_points` alla:
```python
'service = py_srvcli.service_member_function:main',
```

## Ülesanne F: kirjuta client node (kohustuslik)

Samas kaustas loo fail `client_member_function.py`:

```python
import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        return self.cli.call_async(self.req)


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    future = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    rclpy.spin_until_future_complete(minimal_client, future)
    response = future.result()
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

> NB! Ära kasuta `rclpy.spin_until_future_complete` ROS callback'i sees.

## Ülesanne G: lisa client entry point (kohustuslik)

Lõpuks peab `setup.py` `entry_points` olema:
```python
entry_points={
    'console_scripts': [
        'service = py_srvcli.service_member_function:main',
        'client = py_srvcli.client_member_function:main',
    ],
},
```

## Ülesanne H: ehita ja käivita (kohustuslik)

```bash
cd /workspace/ros2_ws
rosdep install -i --from-path src --rosdistro humble -y
colcon build --packages-select py_srvcli
source install/setup.bash
```

**Terminal 1 (service):**
```bash
ros2 run py_srvcli service
```

**Terminal 2 (client — näide):**
```bash
source /workspace/ros2_ws/install/setup.bash
ros2 run py_srvcli client 2 3
```

Peatamiseks: **Ctrl + C** (service terminalis)

---

# Osa 2: Kohandatud liidesed (`tutorial_interfaces`)

Lood **enda ROS 2 liidese definitsioonid** (sõnumid ja teenused) eraldi
paketis ning kasutad neid kahes eelmises paketis: `py_pubsub` ja
`py_srvcli`. Mõlemad paketid peavad asuma samas workspace'is
(`/workspace/ros2_ws/src`).

## Õpiväljundid
- lood `ament_cmake` interface-paketi (`tutorial_interfaces`)
- lisad oma `.msg` ja `.srv` definitsioonid
- seadistad `CMakeLists.txt` ja `package.xml` nii, et liidesed genereeritakse
- kontrollid, et liidesed on avastatavad `ros2 interface show` abil
- uuendad `py_pubsub` ja `py_srvcli` pakette, et kasutada uusi liideseid

## A. Loo uus interface-pakett (kohustuslik)

```bash
cd /workspace/ros2_ws/src
ros2 pkg create --build-type ament_cmake --license Apache-2.0 tutorial_interfaces
cd tutorial_interfaces
mkdir msg srv
```

## B. Loo kohandatud definitsioonid (kohustuslik)

`tutorial_interfaces/msg/Num.msg`:
```text
int64 num
```

`tutorial_interfaces/msg/Sphere.msg`:
```text
geometry_msgs/Point center
float64 radius
```

`tutorial_interfaces/srv/AddThreeInts.srv`:
```text
int64 a
int64 b
int64 c
---
int64 sum
```

## C. Muuda `CMakeLists.txt` (kohustuslik)

Ava `/workspace/ros2_ws/src/tutorial_interfaces/CMakeLists.txt`, lisa:

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

> ⚠️ **Järjekord on oluline!** Need read pead lisama **enne** rida
> `ament_package()`, mis on faili lõpus. Kui lisad selle pärast, saad
> build'i käigus vea `rosidl_generate_interfaces() must be called
> before ament_package()`.

## D. Muuda `package.xml` (kohustuslik)

Lisa `<package>` elemendi sisse:
```xml
<depend>geometry_msgs</depend>
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

## E. Ehita `tutorial_interfaces` (kohustuslik)

```bash
cd /workspace/ros2_ws
colcon build --packages-select tutorial_interfaces
```

## F. Kontrolli, et liidesed on loodud (kohustuslik)

```bash
cd /workspace/ros2_ws
source install/setup.bash
ros2 interface show tutorial_interfaces/msg/Num
ros2 interface show tutorial_interfaces/msg/Sphere
ros2 interface show tutorial_interfaces/srv/AddThreeInts
```

## G. Kasuta uusi liideseid teistes pakettides (kohustuslik)

### G1. Uuenda `py_pubsub` kasutama `Num.msg`

Publisher (`publisher_member_function.py`): asenda `std_msgs.msg.String`
→ `tutorial_interfaces.msg.Num`, `msg.data` → `msg.num`.

```python
from tutorial_interfaces.msg import Num

self.publisher_ = self.create_publisher(Num, 'topic', 10)

msg = Num()
msg.num = self.i
self.get_logger().info('Publishing: "%d"' % msg.num)
```

Subscriber (`subscriber_member_function.py`): samad muudatused.

```python
from tutorial_interfaces.msg import Num

self.subscription = self.create_subscription(Num, 'topic', self.listener_callback, 10)

def listener_callback(self, msg):
    self.get_logger().info('I heard: "%d"' % msg.num)
```

`py_pubsub/package.xml` — lisa: `<exec_depend>tutorial_interfaces</exec_depend>`

Ehita ja käivita:
```bash
cd /workspace/ros2_ws
colcon build --packages-select py_pubsub
source install/setup.bash
ros2 run py_pubsub talker    # terminal 1
ros2 run py_pubsub listener  # terminal 2
```

### G2. Uuenda `py_srvcli` kasutama `AddThreeInts.srv`

- `service_member_function.py`: `AddTwoInts` → `AddThreeInts`, teenuse
  nimi `add_two_ints` → `add_three_ints`, summa `a+b+c`.
- `client_member_function.py`: samad muudatused, loe 3 argumenti
  (nt `client 2 3 1`).
- `py_srvcli/package.xml` — lisa: `<exec_depend>tutorial_interfaces</exec_depend>`

Ehita ja käivita:
```bash
cd /workspace/ros2_ws
colcon build --packages-select py_srvcli
source install/setup.bash
ros2 run py_srvcli service        # terminal 1
ros2 run py_srvcli client 2 3 1   # terminal 2
```

## Kontrollpunktid

- [ ] `py_srvcli` service+client töötavad: `client 2 3` väljastab "Result of add_two_ints: for 2 + 3 = 5".
- [ ] `tutorial_interfaces` build õnnestub ja `ros2 interface show` näitab kõiki kolme liidest.
- [ ] `py_pubsub` kasutab `Num.msg`-i (mitte `std_msgs/String`) ja talker/listener töötavad.
- [ ] `py_srvcli` kasutab `AddThreeInts.srv`-i ja `client 2 3 1` väljastab õige summa (6).

## Esitamine

Paki kokku `ros2_ws/src/py_srvcli`, `ros2_ws/src/tutorial_interfaces` ja
uuendatud `ros2_ws/src/py_pubsub` ning lae ZIP-ina üles Moodle'isse.
