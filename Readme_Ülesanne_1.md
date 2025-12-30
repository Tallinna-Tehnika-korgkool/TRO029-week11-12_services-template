# Week 11–12: Services (Python) – `py_srvcli` (Ülesanne 1/2)

Selle nädala jooksul teed **kaks** eraldi ülesannet. See README kirjeldab **Ülesannet 1/2: Service + Client**.

## Õpiväljundid
- lood ROS 2 Python paketi `ament_python`
- kasutad teenuse tüüpi `example_interfaces/srv/AddTwoInts`
- kirjutad **service node** ja **client node**
- seadistad `entry_points`, et `ros2 run` töötaks
- ehitad ja käivitad teenuse kahes terminalis

## Eeldused
- ROS 2 Humble keskkond töötab (Docker/Devcontainer).
- Uues terminalis on ROS 2 sätitud (nt `source /opt/ros/humble/setup.bash`).

---

## Ülesanne A: loo pakett (kohustuslik)

1) Mine workspace’i `src` kausta:
```bash
cd ~/ros2_ws/src
```

2) Loo pakett nimega **py_srvcli** ning lisa sõltuvused automaatselt:
```bash
ros2 pkg create --build-type ament_python --license Apache-2.0 py_srvcli --dependencies rclpy example_interfaces
```

> `--dependencies` lisab vajalikud read automaatselt `package.xml` faili.

---

## Ülesanne B: uuenda `package.xml` metaandmed (hindeline)

Ava fail:
```
~/ros2_ws/src/py_srvcli/package.xml
```

Täida kindlasti:
```xml
<description>Python client server tutorial</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
```

---

## Ülesanne C: uuenda `setup.py` metaandmed (hindeline)

Ava fail:
```
~/ros2_ws/src/py_srvcli/setup.py
```

Täida `setup()` väljad nii, et need klapiksid `package.xml`-iga:
```python
maintainer='Your Name',
maintainer_email='you@email.com',
description='Python client server tutorial',
license='Apache License 2.0',
```

---

## Ülesanne D: kirjuta service node (kohustuslik)

1) Mine Pythoni paketi kausta:
```bash
cd ~/ros2_ws/src/py_srvcli/py_srvcli
```

2) Loo fail `service_member_function.py` ja kopeeri sisse järgmine kood:

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

---

## Ülesanne E: lisa service entry point (kohustuslik)

Ava `setup.py` (asub `~/ros2_ws/src/py_srvcli/`) ja lisa `entry_points` alla:

```python
'service = py_srvcli.service_member_function:main',
```

---

## Ülesanne F: kirjuta client node (kohustuslik)

Samas kaustas (`~/ros2_ws/src/py_srvcli/py_srvcli`) loo fail `client_member_function.py` ja kopeeri sisse:

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

> NB! Ära kasuta `rclpy.spin_until_future_complete` ROS callback’i sees.

---

## Ülesanne G: lisa client entry point (kohustuslik)

Lisa `setup.py` `entry_points` listi ka:

```python
'client = py_srvcli.client_member_function:main',
```

Lõpuks peab `entry_points` olema selline:

```python
entry_points={
    'console_scripts': [
        'service = py_srvcli.service_member_function:main',
        'client = py_srvcli.client_member_function:main',
    ],
},
```

---

## Ülesanne H: ehita ja käivita (kohustuslik)

1) Mine workspace’i juurkausta:
```bash
cd ~/ros2_ws
```

2) Kontrolli sõltuvusi:
```bash
rosdep install -i --from-path src --rosdistro humble -y
```

3) Ehita pakett:
```bash
colcon build --packages-select py_srvcli
```

4) Source’i setup:
```bash
source install/setup.bash
```

### Käivitamine (2 terminali)

**Terminal 1 (service):**
```bash
ros2 run py_srvcli service
```

**Terminal 2 (client – näide):**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run py_srvcli client 2 3
```

Peatamiseks: **Ctrl + C** (service terminalis)

---

## Esitamise kord

Harjutuse edukaks esitamiseks **pead töötama oma isiklikus GitHub Classroomi repos**, mis on loodud selle nädala jaoks.

1.  **Ava ülesande link Moodle'ist:**
    - Mine kursuse Moodle'i lehele.
    - Leia sealt selle nädala (Week 11-12) ülesande juurest link oma isikliku GitHub Classroomi repositooriumi loomiseks.

2.  **Klooni enda isiklik repo GitHubist**, mille nimi sisaldab sinu GitHubi kasutajanime.
    Näide:
    ```bash
    git clone https://github.com/Tallinna-Tehnika-korgkool/TRO029-week11-12_services-<sinu_kasutajanimi>
    ```

3.  **Tee kõik ülesanded selles kloonitud repos.**

4.  **Lisa ja `commit`'i oma muudatused** (`ros2_ws` kaust koos sinu paketiga).
    ```bash
    git add .
    git commit -m "docs: Complete week 11-12 exercise"
    git push
    ```

5.  Kui `push` on tehtud, käivituvad **automaatsed testid** (GitHub Actions) sinu repo **Actions** vahekaardil.
    - ✅ **Roheline** ✔️ tähendab, et kõik on korras.
    - ❌ **Punane** ✖️ tähendab, et midagi on puudu või valesti – paranda ja tee uus `commit` ja `push`.

> **NB!** Kui töötad väljaspool oma isiklikku Classroom repo't, siis testid ei tööta ja harjutust ei loeta esitatuks.
