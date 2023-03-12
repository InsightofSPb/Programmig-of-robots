# <center> Курсовая работа по дисциплине "Программирование роботов" </center>
# <center> На тему "Управление жестами" </center>


## <center>Команда "мы!", ИТМО</center>
## Состав:
+ Топольницкий Александр
+ Мельникова Виолетта
## Преподаватель - Артёмов Кирилл
<br></br>
## Задачи:
+ Реализовать код, позволяющий выполнять с камеры получать информацию о жесте;
+ Загрузить выбранного робота;
+ Создать мир для робота;
+ Реализовать управление на основе жестов.
<br></br>
## <center><b>Ход работы</b></center>

## Пункт 1. Установка всего необходимого. Загрузка робота
Для начала необходимо, чтобы на системе был установлен ROS noetic. Скачать его можно [по ссылке](http://wiki.ros.org/noetic/Installation/Ubuntu). Лучше устанавлить full-версию, чтобы не пришлось отдельно докачивать какие-либо элементы.\
Выберем следующего робота - `hector quadcopter`.
Дальше создаём папку для нашего проекта и клонируем в неё следующие объекты:
```
mkdir -p ~/drone/src

cd ~/drone/src

git clone https://gitlab.com/beerlab/robot_programming_course/hector_quadrotor.git
```
На случай, если возникнут проблемы с версиями, то пакет с дроном можно взять из следующего источника:
```
git clone https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor.git
```
Теперь необходимо собрать пакет и установить зависимости на папку `src`, чтобы ROS в случае чего сам подтянул из папки необходимые пакеты и обновить окружение. Команды выполняются последовательно в папке `drone`:

```
cd ~/drone
rosdep install --from-paths src --ignore-src -r -y
catkin build
source devel/setup.basg
```
Возможно, придётся установить или обновить `rosdep`, делаем это, затем собираем пакет и обновляем окружение

Теперь если прописать `cd src` и затем выполнить следующую команду 
```
roslaunch hector_quadcopter_gazebo quadcopter_empy_world.launch
```
откроется пустой мир с дроном. 

## Пункт 2. Работа с симулятором
Для дальнейшего удобства управления роботом было бы неплохо не только видеть его самого, но и наблюдать картину от его лица, чтобы своевременно принимать решения.\
Для этого зайдём в папку `hector_quadcopter_gazebo`, в `launch`, в `quadcopter_empy_world.launch`. Файл должен выглядеть так:
```html
<?xml version="1.0"?>

<launch>
  <arg name="paused" default="false"/>
  <arg name="use_sim_time" default="true"/>
  <arg name="gui" default="true"/>
  <arg name="headless" default="false"/>
  <arg name="debug" default="false"/>

  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="paused" value="$(arg paused)"/>
    <arg name="use_sim_time" value="$(arg use_sim_time)"/>
    <arg name="gui" value="$(arg gui)"/>
    <arg name="headless" value="$(arg headless)"/>
    <arg name="debug" value="$(arg debug)"/>
  </include>

  <include file="$(find hector_quadrotor_gazebo)/launch/spawn_quadrotor.launch" />
</launch>
```

Найдём строку 
```html
 <include file="$(find hector_quadrotor_gazebo)/launch/spawn_quadrotor.launch" />
```
И заменим её на
```html
 <include file="$(find hector_quadrotor_gazebo)/launch/spawn_quadrotor_with_cam.launch" />
```
Что нам это дало? Мы заменили модель робота без камеры на робота с камерой. Сохранить. \
Теперь нам необходимо создать свой мир в Gazebo. Это можно сделать двумя способами:
1. Открыть пустой мир, прописав `roslaunch hector_quadcopter_gazebo quadcopter_empy_world.launch`, нажать слева сверху `Edit` -> `Building Editor`. Откроется окошко, в котором можно добавлять стены, окна, лестницы. Если сохранить этот мир, то его потом можно будет использовать для своих симуляций, однако необходимо привести его к виду, который будет описан ниже.
2. Создать пустой файл, например, `world.world` и прописать в него следующее:
```html
<?xml version='1.0'?>
<sdf version='1.7'>
  <world name="default">
    <include>
      <uri>model://sun</uri>

    </include>

    <include>
      <uri>model://ground_plane</uri>
    </include>
  </world>
</sdf>
```
Мы добавили базовые элементы - солнце и землю (`ground_plane`), которая нужна, чтобы робот не падал сквозь карту. Мною в симуляторе были добавлены модельки, их можно загрузить в этот файлик. Всё, что между тегами <model> и </model> было получено с помощью первого варианта. Итоговый код выглядит следующим образом:
```html
<?xml version='1.0'?>
<sdf version='1.7'>
  <world name="default">
    <include>
      <uri>model://sun</uri>

    </include>

    <include>
      <uri>model://ground_plane</uri>
    </include>
    
  <model name='Drone_trace'>
    <pose>0.491127 -1.10045 0 0 -0 0</pose>
    <link name='Wall_1'>
      <pose>-3.45213 0.561447 0 0 -0 2.35619</pose>
      <visual name='Wall_1_Visual_0'>
        <pose>-1.46661 0 2.25 0 -0 0</pose>
        <geometry>
          <box>
            <size>1.06678 0.15 4.5</size>
          </box>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Bricks</name>
          </script>
          <ambient>1 1 1 1</ambient>
        </material>
        <meta>
          <layer>0</layer>
        </meta>
      </visual>
      <collision name='Wall_1_Collision_0'>
        <geometry>
          <box>
            <size>1.06678 0.15 4.5</size>
          </box>
        </geometry>
        <pose>-1.46661 0 2.25 0 -0 0</pose>
      </collision>
      <visual name='Wall_1_Visual_1'>
        <pose>0.533392 0 0.25 0 -0 0</pose>
        <geometry>
          <box>
            <size>2.93322 0.15 0.5</size>
          </box>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Bricks</name>
          </script>
          <ambient>1 1 1 1</ambient>
        </material>
        <meta>
          <layer>0</layer>
        </meta>
      </visual>
      <collision name='Wall_1_Collision_1'>
        <geometry>
          <box>
            <size>2.93322 0.15 0.5</size>
          </box>
        </geometry>
        <pose>0.533392 0 0.25 0 -0 0</pose>
      </collision>
      <visual name='Wall_1_Visual_2'>
        <pose>1.78339 0 2.5 0 -0 0</pose>
        <geometry>
          <box>
            <size>0.433216 0.15 4</size>
          </box>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Bricks</name>
          </script>
          <ambient>1 1 1 1</ambient>
        </material>
        <meta>
          <layer>0</layer>
        </meta>
      </visual>
      <collision name='Wall_1_Collision_2'>
        <geometry>
          <box>
            <size>0.433216 0.15 4</size>
          </box>
        </geometry>
        <pose>1.78339 0 2.5 0 -0 0</pose>
      </collision>
      <visual name='Wall_1_Visual_3'>
        <pose>0.316784 0 3.4 0 -0 0</pose>
        <geometry>
          <box>
            <size>2.5 0.15 2.2</size>
          </box>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Bricks</name>
          </script>
          <ambient>1 1 1 1</ambient>
        </material>
        <meta>
          <layer>0</layer>
        </meta>
      </visual>
      <collision name='Wall_1_Collision_3'>
        <geometry>
          <box>
            <size>2.5 0.15 2.2</size>
          </box>
        </geometry>
        <pose>0.316784 0 3.4 0 -0 0</pose>
      </collision>
    </link>
    <link name='Wall_11'>
      <collision name='Wall_11_Collision'>
        <geometry>
          <box>
            <size>0.75 0.15 7.5</size>
          </box>
        </geometry>
        <pose>0 0 3.75 0 -0 0</pose>
      </collision>
      <visual name='Wall_11_Visual'>
        <pose>0 0 3.75 0 -0 0</pose>
        <geometry>
          <box>
            <size>0.75 0.15 7.5</size>
          </box>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Grey</name>
          </script>
          <ambient>0.435294 0.796078 0.67451 1</ambient>
        </material>
        <meta>
          <layer>0</layer>
        </meta>
      </visual>
      <pose>4.60147 -4.74672 0 0 -0 -1.0472</pose>
    </link>
    <link name='Wall_5'>
      <collision name='Wall_5_Collision'>
        <geometry>
          <box>
            <size>5 0.15 1</size>
          </box>
        </geometry>
        <pose>0 0 0.5 0 -0 0</pose>
      </collision>
      <visual name='Wall_5_Visual'>
        <pose>0 0 0.5 0 -0 0</pose>
        <geometry>
          <box>
            <size>5 0.15 1</size>
          </box>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Wood</name>
          </script>
          <ambient>1 1 1 1</ambient>
        </material>
        <meta>
          <layer>0</layer>
        </meta>
      </visual>
      <pose>3.60437 2.90645 0 0 -0 2.09442</pose>
    </link>
    <static>1</static>
  </model>
  </world>
</sdf>
```

Теперь нам необходимо добавить наш новый мир в запускаемый файл. Для этого между строками
```html
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
   
    <arg name="paused" value="$(arg paused)"/>
```
Вписать (на одном уровне с другими арг. нейм)
```html

    <arg name="world_name" value="$(find hector_quadrotor_gazebo)/launch/world.world"/>
   
```
Итоговый `launch-file`:
```html
<?xml version="1.0"?>

<launch>
  <arg name="paused" default="false"/>
  <arg name="use_sim_time" default="true"/>
  <arg name="gui" default="true"/>
  <arg name="headless" default="false"/>
  <arg name="debug" default="false"/>

  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find hector_quadrotor_gazebo)/launch/world.world"/>
    <arg name="paused" value="$(arg paused)"/>
    <arg name="use_sim_time" value="$(arg use_sim_time)"/>
    <arg name="gui" value="$(arg gui)"/>
    <arg name="headless" value="$(arg headless)"/>
    <arg name="debug" value="$(arg debug)"/>
  </include>

  <include file="$(find hector_quadrotor_gazebo)/launch/spawn_quadrotor_with_cam.launch" />
</launch>
```
Если его теперь запустить, то в симуляторе помимо дрона должны появиться кирпичная стена с окошком, деревянная длинная преграда и зелёная колонна.

## Пункт 3. Написание кода для считывания жестов
За основу взят код, представленный по [адресу](https://www.youtube.com/watch?v=NZde8Xt78Iw). Однако, необходимо некоторые строчки изменить в отличие от оригинала, поскольку с тех пор обновилась используемая библиотека. Эти строчки приведены ниже, а дальше будут сам код с комментариями:
```python
class handDetector():
    def __init__(self, mode=False, maxHands = 2, detectionCon = 0.5, trackCon=0.5, modComp = 1):
        self.mode = mode
        self.maxHands = maxHands
        self.modComp = modComp
        self.detectionCon = detectionCon
        self.trackCon = trackCon

        
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.maxHands,self.modComp,self.detectionCon,
                                        self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils
```
От оригинала отличается аргументом `modComp`. Необходимо также прописать атрибут `self.modComp` и в `self.hands` вставить этот атрибут после `self.maxHands`. Если вставить в другое место, будет ошибка. Теперь приведу код с комментариями.

### Импортируем все необходимые библиотеи
```python
import cv2
import time
import mediapipe as mp
import math
import numpy as np


```
## Создаём класс handDetector()
```python
class handDetector():
    def __init__(self, mode=False, maxHands = 2, detectionCon = 0.5, trackCon=0.5, modComp = 1):
        self.mode = mode
        self.maxHands = maxHands  # максимальное количество рук
        self.modComp = modComp  
        self.detectionCon = detectionCon  # задаём уверенность обнаружения рук 
        self.trackCon = trackCon

        #  обращаемся к методам библиотеки и задаём атрибуты для последующей работы
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.maxHands,self.modComp,self.detectionCon,
                                        self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils
```
### Добавляем метод по поиску рук
```python
    def findHands(self, img, suc=True, draw=True):
        if suc == True:
            imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # перевод изображения
            self.results = self.hands.process(imgRGB)  # создаём атрибут по вычислению (поиску) рук
            # print(results.multi_hand_landmarks)

        
        # определяем марку для каждого элемента (на каждом пальце несколько точек)
        # можно найти в Интернете по поиску "mediapipe hands", картинки будут
        if self.results.multi_hand_landmarks:  
            for handLms in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(img, handLms, self.mpHands.HAND_CONNECTIONS)
        return img
```
### Добавляем метод для определения положения каждой марки для каждого пальца
```python
   def findPosition(self, img, handNo=0, draw=True):
        
        self.lmList = []
        if self.results.multi_hand_landmarks:
            myHand = self.results.multi_hand_landmarks[handNo]
            for id, lm in enumerate(myHand.landmark):
                # print(id,lm)
                h, w, c = img.shape  # координаты задаём через размеры получаемой картинки
                cx, cy = int(lm.x*w), int(lm.y*h)
                # print(id, cx, cy)
                self.lmList.append([id, cx, cy])
                if draw:
                    cv2.circle(img, (cx, cy), 5, (255,0, 255), cv2.FILLED)  # помечаем каждую марку
        
        
        return self.lmList
```
### Будем задавать управление по высоте и далее следующим образом
Выбираем 2 пальца, считаем расстояние между ними, помечаем их каким-нибудь цветом и соединяем линией. Дальше для более качественного и удобного управления интерполируем расстояние между пальцами в нужное нам.\
Так, в этом фрагменте кода, мы выбираем точки 0 и 20. Это основание ладони и кончик мизинца. Расскрашиваем их в жёлтый цвет. Далее, смотрим, какое расстояние в крайних положениях. Оно было от 70 до 320, но, чтобы не напрягать руку, лучше диапазон уменьшить. Дальше задаём желаемый диапазон высот / других координат
```python
    def findDistanceAlt(self, img, p1=0, p2=20, draw=True):
        x1, y1 = self.lmList[p1][1], self.lmList[p1][2]
        x2, y2 = self.lmList[p2][1], self.lmList[p2][2]
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

        if draw:
            cv2.circle(img, (x1, y1), 10, (0, 255, 255), cv2.FILLED)
            cv2.circle(img, (x2, y2), 10, (0, 255, 255), cv2.FILLED)
            cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 3)
            cv2.circle(img, (cx, cy), 10, (0, 255, 255), cv2.FILLED)

        lengthZ = math.hypot(x2 - x1, y2 - y1)
        lengthZ = np.interp(lengthZ, [100, 280], [0, 3])
        return lengthZ
``` 
### Управление по скорости по Х с помощью среднего пальца и основания ладони
```python
   def findDistanceX(self, img, p1=0, p2=12, draw=True):
        x1, y1 = self.lmList[p1][1], self.lmList[p1][2]
        x2, y2 = self.lmList[p2][1], self.lmList[p2][2]
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

        if draw:
            cv2.circle(img, (x1, y1), 10, (220, 220, 0), cv2.FILLED)
            cv2.circle(img, (x2, y2), 10, (220, 220, 0), cv2.FILLED)
            cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 3)
            cv2.circle(img, (cx, cy), 10, (220, 220, 0), cv2.FILLED)

        lengthX = math.hypot(x2 - x1, y2 - y1)
        lengthX = np.interp(lengthX, [150, 250], [0, 1])
        return lengthX
```
### Управление по угловой скорости wz с помощью кончика большого пальца и точки в середине указательного пальца
```python
    def findDistanceWZ(self, img, p1=4, p2=6, draw=True):
        x1, y1 = self.lmList[p1][1], self.lmList[p1][2]
        x2, y2 = self.lmList[p2][1], self.lmList[p2][2]
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

        if draw:
            cv2.circle(img, (x1, y1), 10, (155, 155, 155), cv2.FILLED)
            cv2.circle(img, (x2, y2), 10, (155, 155, 155), cv2.FILLED)
            cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 3)
            cv2.circle(img, (cx, cy), 10, (155, 155, 155), cv2.FILLED)

        lengthwz = math.hypot(x2 - x1, y2 - y1)
        lengthwz = np.interp(lengthwz, [45, 110], [0, 0.5])
        return lengthwz
```
### Поскольку код является самостоятельным, его можно запустить прямо так. 
### Руку подносить до запуска скрипта!!!
```python
def main():
    pTime = 0
    cTime = 0
    cap = cv2.VideoCapture(0)  # задаём нашу камеру (может быть под цифрой 1 или даже -1)
    detector = handDetector()  # создаём объект класса

    while True:
        success, img = cap.read()  # считываем изображение
        img = detector.findHands(img) # получаем через наш метод
        lmList = detector.findPosition(img) 
        lengthZ = detector.findDistanceAlt(img) # получаем значение для высоты
        lengthX = detector.findDistanceX(img) # получаем значение для скорости по Х
        lengthwz = detector.findDistanceWZ(img) # значение для скорости
        if len(lmList) !=0: # чтобы не выдавало ошибку; наш код работает, когда получает изображение
            print(f'Высота: {lengthZ}, Скорость по Х: {lengthX}, Скорость поворота: {lengthwz}')
        
        #  для вывода fps
        cTime = time.time()
        fps = 1/(cTime - pTime)
        pTime = cTime
    
    
        cv2.putText(img, str(int(fps)), (10,70), cv2.FONT_HERSHEY_PLAIN,3, (255,0,255),3)       
        
            
        cv2.imshow('Image',img)
        cv2.waitKey(1)
```
### Добавляем точку входа в скрипт
```python
if __name__ == "__main__":
    main()
```

## Пункт 4. Добавление модуля по определению рук в управление квадрокоптером
За основу возьмём код с прошлого семестра по данному курсу и включим в него написанное управление
### Импортируем библиотеки и наш модуль HandDetection
```python
import rospy
import cv2
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from hector_uav_msgs.srv import EnableMotors
from sensor_msgs.msg import Image

import HandDetection as htm
import numpy as np
```
### Задаём коэффициенты для управления нашими ПД регуляторами
```python
bridge = CvBridge()

Kz = 0.5
Bz = 0.5


Kw = 1
Bw = 1

Kx = 1
Bx = 0.3
```
### Создание класса
Создаём наш класс, добавляем конструктор. Задаём несколько атрибутов
```python
class Contoller:

    def __init__(self):
        rospy.init_node("controller_node")  # инициализируем управляющую ноду
        self.enable_motors()
        
        
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)  # создаём паблишер
        
        rospy.Subscriber("/ground_truth/state", Odometry, self.state_callback)  # подписываемся на данные о положении дрона
        
        self.position = Point()
        self.twist = Twist()
        
        rospy.Subscriber("/cam_2/camera/image", Image, self.camera_front_callback)  #  подписываемся на данные с камеры   
        self.bridge = CvBridge()
        self.omega_error = 0
        self.omega_error_prev = 0

        self.x_error = 0
        self.x_error_prev = 0
        self.cam_front = []
        self.rate=rospy.Rate(50)  # задаём частоту обновления
```
### Добавляем атрибуты на основе нашего модуля
```python
        self.detector = htm.handDetector()  # создаём объект нашего класса handDetector из модуля и делаем его атрибутом текущего класса
        self.cap = cv2.VideoCapture(0)  #  создаём атрибут, связанный с камером
        wCam, hCam = 640, 480  #
        self.cap.set(3, wCam)
        self.cap.set(4, hCam)
```
### Методы, написанные в прошлом семестре, для включения двигателей, получения данных о положении, вида с камеры и отображения вида с камеры
```python
    def enable_motors(self):
        try:
            rospy.wait_for_service('/enable_motors')
            call_em = rospy.ServiceProxy('/enable_motors', EnableMotors)
            resp1 = call_em(True)

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def state_callback(self, msg):
        self.position = msg.pose.pose.position
        self.twist = msg.twist.twist

    def camera_front_callback(self, msg):
        try:
            self.cam_front = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))


    def images(self, img, title):
        cv2.imshow(title, img)
        cv2.waitKey(3)   
```

### Основной цикл spin
```python
    def spin(self):

        while not rospy.is_shutdown():

            try:            
                success, img = self.cap.read()  # через методы нашего атрибута получаем данные
                img = self.detector.findHands(img)
                lmList = self.detector.findPosition(img)
                z_des = self.detector.findDistanceAlt(img)
                x_des = self.detector.findDistanceX(img)
                wz_des = self.detector.findDistanceWZ(img)
   
                # применяем полученные данные для управления      
                uz = Kz * (z_des - self.position.z) - Bz * self.twist.linear.z
                uwz = Kw * (wz_des - self.omega_error) - Bw * (self.omega_error - self.omega_error_prev) / (1.0 / 50.0)
                self.omega_error_prev = self.omega_error
                
                ux = Kx * (x_des - self.x_error) - Bx * (self.x_error - self.x_error_prev) / (1.0 / 50)
                self.x_error_prev = self.x_error
                
                cmd_msg = Twist()

                cmd_msg.linear.x = ux
                cmd_msg.linear.y = 0
                cmd_msg.linear.z = uz
                cmd_msg.angular.z = -uwz
                
                
                print(f'Желаемая высота: {z_des}, Скорость по Х: {ux}, Скорость поворота: {uwz}')
                
                # публикуем в топик скорости данные
                self.cmd_pub.publish(cmd_msg)                
                
                # отображаем камеру с рукой
                cv2.imshow('Hand', img)
                cv2.waitKey(3)  
                
                # отображаем вид с дрона
                if len(self.cam_front > 0):
                    self.images(self.cam_front, title='Front')
                
                self.rate.sleep()



            except KeyboardInterrupt:
                break
```

### Делаем точку входа и запускаем наш объект с методом spin
```python
def main():
    ctrl = Contoller()
    ctrl.spin()


if __name__=='__main__':
    main()
```

### Дополнение. Если `cv2` выдаёт ошибки, возможно два решения:
1. Не подключена камера к виртуальной ОС. Необходимо либо скачать с сайта OrcleVM плагин, либо через меню "Устройства" -> "Веб-камеры" выбрать нужную камеру. Либо поменять цифру в `VideoCapture`. В коде есть пометка
2. Камера может не распознавать руку, так как она слишком близко / далеко. Необходимо поднести руку до запуска скрипта, иначе он сразу же выключится. Возможно также изменения параметра `detection`, позволяющего повысить / понизить уверенность в том, что на камере видна рука.
<br></br>
## Пункт 5. Запуск
Для успешного запуска выполняем следующие шаги:
1. Прописываем в одной вкладке, предварительно обновив окружение через `source devel/setup.bash`. После `roslaunch` должен запуститься мир с квадрокоптером и 3 препятствиями.
```
roslaunch hector_quadcopter_gazebo quadcopter_empy_world.launch
```
2. Открываем новую вкладку в консоли, в директории /drone прописываем
```
source devel/setup.bash
```
3. Переходим в папку `src`, прописываем
```
python3 contoller.py
```
4. Если всё было выполнено последовательно и правильно, то запустятся два окошка - одно с Вашей рукой и марками, второе - вид с камеры дрона. Можно управлять.

## Управление
+ Расстояние между кончиком большого пальца и среней костяшкой указательного отвечает за поворот вокруг оси Oz;
+ Расстояние между мизинцем и основанием ладони за высоту полёта;
+ Расстояние между кончиком среднего пальца и основание ладони за скорость по Ох.