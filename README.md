Всем привет! Давно хотели собрать своего робота? Не слышали про ROS? Не знаете с чего начать?

Тогда вы не зря зашли к нам, мы постараемся максимально подробно и быстро рассказать о нашем пути создания робота на ROS2 Jazzy и esp32,по этим материалам вы с легкостью сможете повторить этот проект. 

Ну а если вы хотите с головой погрузится в мир увлекательной робототехники и разобраться во всех тонкостях, то проходите курс на stepik - https://stepik.org/course/221157/syllabus Все материалы, используемые в этой статье были взяты из этого курса и мы лишь делимся опытом создания робота и проблемами с которыми столкнулись мы. 

Проблемы при сборки робота - это неотъемлемая часть процесса, поэтому мы постарались описать решение всех проблем с которыми мы сталкивались в процессе сборки и программирования робота.

Также много интересной, полезной и актуальной информации можете найти в нашем TG-канале https://t.me/RoboForc 

Содержание:

[I МОДУЛЬ - СБОРКА РОБОТА](#i-модуль---сборка-робота)
[1. ПОДГОТОВКА МАТЕРИАЛОВ И ИНСТРУМЕНТОВ](#1-подготовка-материалов-и-инструментов)
[2. ИЗГОТОВЛЕНИЕ КОРПУСА](#2-изготовление-корпуса)
[Шаг 1: РЕЗКА ДЕТАЛЕЙ](#шаг-1--резка-деталей)
  - [Шаг 2: 3D-ПЕЧАТЬ](#шаг-2--3d---печать)
  - [Шаг 3: СБОРКА КАРКАСА (ПЕРВЫЙ СЛОЙ)](#шаг-3--сборка-каркаса-первый-слой)
  - [Шаг 4: СБОРКА КАРКАСА (СРЕДНИЙ СЛОЙ)](#шаг-4--сборка-каркаса-средний-слой)
  - [Шаг 5: СОЕДИНЕНИЕ НИЖНЕЙ И СРЕДНЕЙ ПЛАТФОРМ](#шаг-5--соединение-нижней-и-средней-платформ)
  - [Шаг 6: ТЕСТОВЫЙ ЗАПУСК](#шаг-6--тестовый-запуск)
[II МОДУЛЬ - ПРОГРАММИРОВАНИЕ ESP32](#ii-модуль---программирование-esp32)

# I МОДУЛЬ - СБОРКА РОБОТА

# 1. ПОДГОТОВКА МАТЕРИАЛОВ И ИНСТРУМЕНТОВ
Необходимые инструменты:
- Лазерный резак – для резки фанеры
- Дрель и сверла (2–3мм) – для отверстий
- Набор отверток с разными насадками
- Паяльник (40 Вт) + припой (POS-60) – для электроники
- Мультиметр – проверка цепей
- Наждачка - для обработки краёв деталей после резки.

Материалы и комплектующие:  

| Компонент | Описание | Кол-во | Ссылка | Стоимость |
| ------- | ------- | ------- | ------- |------- |
| Фанера | Толщина 6 мм, 600х600 | 1 лист | https://www.ozon.ru/product/fanera-9-mm-sort-3-3-600h600-mm-1869700011/?at=PjtJzQPQXcWjXyvmIxmY3MwS95o8Lzcq71D1mFPGRX8v&from_sku=1901529745&oos_search=false | 370 |
| Винты М3 (+ гайки М3)| 30шт х 20мм, 4шт х 30мм, 4шт х 50мм | 38 | https://aliexpress.ru/item/1005005552471765.html?sku_id=12000033513521183&spm=a2g2w.productlist.search_results.15.7c287835NR6JBS | 160 |
| Винты М2.5 (+ гайки М2.5) | 2шт х 10мм | 2 | https://aliexpress.ru/item/1005005552471765.html?sku_id=12000033513521145&spm=a2g2w.productlist.search_results.15.7c287835NR6JBS | 150 |
| Винты М2 (+ гайки М2)| 4 шт х 10мм | 4 | https://aliexpress.ru/item/1005005552471765.html?sku_id=12000033513521113&spm=a2g2w.productlist.search_results.15.7c287835NR6JBS | 120 |
| Муфты  | Необходимы для закрепления колес на валах моторов | 2 | https://aliexpress.ru/item/1005004333022107.html?sku_id=12000028792464891&srcSns=sns_More&businessType=ProductDetail&spreadType=socialShare&tt=MG&utm_medium=sharing | 420 |
| OMNI-колесо | Позволяет двигаться роботу в любом направлении ьез поворота основных колес | 1 | https://aliexpress.ru/item/1005008405743640.html?sku_id=12000045061747881&spm=a2g2w.productlist.search_results.11.5e415abdiln3SR | 1540 |
| Микрокомпьютер Raspberry pi 5 8gb | «Верхний уровень». Обрабатывает информацию, поступающую с ESP (“нижнего уровня»). Поддержка новейших интерфейсов. Отлично подходит к задачам компьютерного зрения, обработки трёхмерных данных с лидаров.  | 1 | https://sl.aliexpress.ru/p?key=0vxkGbg | 10000 |
| Микроконтроллер ESP32-WROOM-32 | «Нижний уровень». Собирает информацию с датчиков, энкодеров и отправляет на «верхний уровень». Подходит для сетевого проекта из-за встроенных Wi-Fi и Bluetooth, высокой производительности и большого объёма памяти. | 1 | https://sl.aliexpress.ru/p?key=7oxkGZr | 300 |
| Моторы JGA25-370 77 RPM 12 B | Необходимы для передвижения робота. Используем 12В для упрощения питания. | 2 | https://aliexpress.ru/item/1005007793208347.html?sku_id=12000042220805718&spm=a2g2x.productlist.search_results.1.2ccd33e3p7kOox | 1200 |
| Драйвер моторов L298N | Необходим для управления моторов, требующих высокое напряжение. | 1 | https://aliexpress.ru/item/1005007125159518.html?sku_id=12000042919988210&srcSns=sns_More&businessType=ProductDetail&spreadType=socialShare&tt=MG&utm_medium=sharing | 150 |
| Понижающие DC-DC преобразователи Readytosky FPV RC UBEC 5V 5A | Обеспечивает стабильное питание для микроконтроллера, микрокомпьютера, датчиков и драйверов. | 2 | https://aliexpress.ru/item/33019505937.html?sku_id=67198302173&srcSns=sns_More&businessType=ProductDetail&spreadType=socialShare&tt=MG&utm_medium=sharing | 530 |
| Литиевые батареи JOUYM 18650, 2000 мАч | Обеспечивает питание робота. Обладают высокой плотностью энергии при относительно небольшом размере. Можно последовательно соединять несколько штук, добиваясь нужного напряжения и ёмкости. | 3 | https://sl.aliexpress.ru/p?key=ajxkGkw | 460 |
| Держатель для батарей | Корпус для уставноки литейных батареек | 1 | https://aliexpress.ru/item/1005005586103501.html?sku_id=12000033652145033&spm=a2g2w.productlist.search_results.2.476f1d1frrOOTL | 140 |
| Лидар SLAMTEC RPLIDAR C1 | «Глаза» робота, который смотрит на окружение, вычисляет расстояние, определяет препятствия впереди | 1 | https://sl.aliexpress.ru/p?key=fmxkGwR | 6740 |
| Штыревые штекеры и гнезда | Служат для удобства работы с микроконтроллером, его легкой замены - провода припаиваются не к самому МК, а к штекерам. | 2х2 | https://aliexpress.ru/item/1005004954771161.html?spm=a2g2x.detail.rcmdprod.7.6d172dc9ZId1M8&mixer_rcmd_bucket_id=UnknownMixerAbId&pdp_trigger_item_id=0_1005005576560025&ru_algo_pv_id=f712b2-61b690-22faee-72e977-1743876000&scenario=aerSimilarItemPdpRcmd&sku_id=12000031142220245&traffic_source=recommendation&type_rcmd=core | 200 |
| Комплект разъемов | Нужны для соединения электронной схемы без пайки. | 1 | https://aliexpress.ru/item/1005007622670523.html?sku_id=12000041540582736&spm=a2g2w.productlist.search_results.10.2e79633bJ9Bw1E | 120 |

СТОИМОСТЬ КОМПОНЕНТОВ ДЛЯ СБОРКИ РОБОТА:

За относительно небольшой бюджет можно также собрать достаточно надежного и производительного робота.
Суммируя все цены комплектующих из I модуля 1 пункта, мы получаем итоговую стоимость нашей модели - 22.600 рублей (на момент 06.04.2025).

# 2. ИЗГОТОВЛЕНИЕ КОРПУСА

# Шаг 1: РЕЗКА ДЕТАЛЕЙ
Загрузите DXF-файлы в станок лазерной резки.  
Основные детали:
- Основание (нижняя платформа)
- Средняя платформа
- Крышка (верхняя платформа)
- 2 опоры для OMNI колеса
- 5 стоек по 60 мм
- 4 стойки по 130 мм
После резки обработайте края наждачкой, чтобы избежать заусенцев.  

# Шаг 2: 3D-ПЕЧАТЬ
Загрузите stl-файлы в станок 3D-печати.  
Основные детали:  
- 2 колеса
- Стойки для моторчиков
![image](https://github.com/user-attachments/assets/6dffb7dc-7430-4fb7-bbe7-f751f1ad7de7)
![image](https://github.com/user-attachments/assets/e265e2c7-af5c-46d4-87a9-87122f94bcaa)

# Шаг 3: СБОРКА КАРКАСА (ПЕРВЫЙ СЛОЙ)
1. Установка мотор редукторов
На валы мотор-редукторов установите колёса с помощью муфт (2 шт) и винтов М3 (2 шт), закрепите их стопорными винтами.
![image](https://github.com/user-attachments/assets/9f04b441-88c4-4640-9b63-5f7a0e0fc872)

Найдите на нижней платформе посадочные места под моторы.
Зафиксируйте каждый мотор винтами M3 х 50 мм (4 шт) и гаек М3 (4 шт) с помощью напечатанных стоек (обратите внимание на то, чтобы провода от энкодеров были выведены к центру платформы)
Убедитесь, что при вращении колёса не задевают за край платформы и установлены ровно.
Уделите внимание ровной установке колес, это обеспечит качественное движение без лишних корректировок.
![image](https://github.com/user-attachments/assets/dd758873-1871-4d8a-adda-b4ce5c991556)
![image](https://github.com/user-attachments/assets/e09da91f-3ad2-423f-b8ae-80c137f1c8ae)

2. Установка OMNI колеса 
Найдите на нижней платформе посадочные места под вырезанные опоры OMNI колеса.
Установите их с помощью винтов М3 х 20 (4 шт) и гаек М3 (4 шт)
![image](https://github.com/user-attachments/assets/01de58ea-3ed2-43d3-945e-6c7934180a54)
![image](https://github.com/user-attachments/assets/226cf342-804d-4183-8706-759e79d707e0)

Закрепите OMNI колесо винтами М3 (2 шт).
Убедитесь, что при вращении колёсо не задевает за край платформы.

3. Установка драйвера моторов
Найдите в центре нижней платформы посадочные места под драйвер
Установите драйвер с помощью винтов М3 (2шт) и гаек М3 (2шт)
![image](https://github.com/user-attachments/assets/9039d366-f1d5-4f64-aa69-340d8d99fd79)

# Шаг 4: СБОРКА КАРКАСА (СРЕДНИЙ СЛОЙ)
1. Пайка шилда для ESP
Шилд – плата с контактными площадками.   
Установите на ESP штыревые гнезда, затем вставьте конструкцию в печатную плату (со стороны пайки)
![image](https://github.com/user-attachments/assets/27ad958d-71a9-48fe-be3d-f2073585798e)
Установите штыревые штекеры в средних с конструкцией линиях на плате.
Аккуратно переверните плату, поддерживая все компоненты.
![image](https://github.com/user-attachments/assets/7b1cd3d8-278e-419c-9962-18710bfe69ff)

Включите паяльник и дождитесь, пока инструмент разогреется до 300-350°C. 
Нанесите припой на крайние контакты (используйте флюс для лучшего растекания припоя)
Необходимо спаять все крайние контакты попарно (контакт гнезда с контактом штекера)
Проведите аналогичную работу со всеми оставшимися контактами (ВАЖНО: два спаянных контакты не должны быть соединены с соседними. 
![image](https://github.com/user-attachments/assets/fb3f67cf-30df-44e3-9284-4bc72aa49e9a)

После окончания пайки необходимо провести проверку: 
А. Визуальный осмотр:  
   - Нет ли перемычек между контактами (проверка на отсутствие короткого замыкания)
 Все ли дорожки пропаяны (особенно у штыревых разъёмов).  
Б.  Тест мультиметром:  
   - Прозвоните линии 5V и 3.3V на КЗ.  
   - Проверьте контакты GPIO на обрыв.

2. Установка шилда на платформу
Найдите на средней платформе посадочное место для шилда
Вставьте снизу в платформу винты М2 (4шт) и накрутите на них гайки М2 (4шт). Данная конструкция не позволит передать находящиеся под шилдом провода  
Установите на винты собранный шилд и закрепите его сверху гайками М2 (4шт)
![image](https://github.com/user-attachments/assets/030c5e43-1274-4afe-9a40-431a16b0ff15)

3. Установка блока питания
Найдите на средней платформе посадочное место для блока питания (средняя часть платформы) 
Закрепите блок с помощью винтов М3 (2 шт) и гаек М3 (2шт)  (ВАЖНО: провода должны выходить в центральную часть платформы)
![image](https://github.com/user-attachments/assets/b63c3e8f-8db2-4674-855f-9d3161c8c713)

4. Монтаж:
   макетную и монтажную схему можно подробно изучить в папке Electronic scheme
Электрическая схема:
![Uploading image.png…]()

Пайка проводов, преобразователей, RGB ленты, драйвера, энкодеров, микроконтроллера.
Залудите контакты перед пайкой, а затем работайте по тому же принципу, как и при пайке шилда  
После окончания пайки необходимо провести проверку (визуальный осмотр и тест мультиметром) (ВАЖНО: проверьте, нет ли пережатых проводов)
![image](https://github.com/user-attachments/assets/de35dbd0-492c-492b-9d6c-5eb8d78bf625)
![image](https://github.com/user-attachments/assets/2defbdcc-f68b-4a34-892a-fc94218b6f51)

# Шаг 5: СОЕДИНЕНИЕ НИЖНЕЙ И СРЕДНЕЙ ПЛАТФОРМ
Соберите стойки из фанеры: 5 штук по 60 мм, 4 штуки по 130 мм (для этого понадобятся гайки М3 18 шт)
Соедините нижнюю и верхнюю платформы с помощью стоек по 60 мм, винтов М3 (10шт)
![image](https://github.com/user-attachments/assets/f270cfb1-d21b-4bdc-842e-68b68ae3c9c5)
![image](https://github.com/user-attachments/assets/9d75c9f5-c1b5-4cfd-8cf2-11af3c26be24)
![image](https://github.com/user-attachments/assets/ef5b23e3-8283-4068-adc9-62259d38eddd)
![image](https://github.com/user-attachments/assets/f2c281f6-5179-434e-a022-b95a987e2805)
![image](https://github.com/user-attachments/assets/67f2db67-e9ac-4b81-8ecf-74aa50b34ac0)

# Шаг 6: ТЕСТОВЫЙ ЗАПУСК
- Подключите питание. 
- Проверьте работу моторов, питание и загрузку ESP.
- Проанализируйте, движится ли робот в заданном вами направлении или нет.
ВАЖНО: есть вероятность, что робот поедет в обратную сторону, так как у него 2 шнура, которые можно перепутать местами. В таком случае необходимо поменять местами номера бортов в коде и загрузить его. Теперь робот поедет в заданном вами направлении.
- Измерьте количество тиков и запишите их в программу.
![image](https://github.com/user-attachments/assets/0a724450-055a-4aee-9a32-913ad60914eb)
![image](https://github.com/user-attachments/assets/680014d7-5151-4f43-a8e2-11e9fd9dd73b)
![Uploading image.png…]()


# II МОДУЛЬ - ПРОГРАММИРОВАНИЕ ESP32

После того как вся аппаратная часть собрана, можно переходить и к программной.
Все действия лучше делать на ноутбуке - так как это будет быстрее и надежней, хотя в теории можете и сделать все тоже самое на raspberry.
Начнем с программирования "нижнего уровня", тоесть esp32. Программировать будем через arduino IDE (если она еще у вас не установлена, то можете скачать с официального сайта https://www.arduino.cc/en/software ).
Чтоб можно было прошивать нашe платe добавим ее в список доступных для прошивки. Нужно зайти в файл->настройки и в поле "Дополнительные менеджеры плат" вставляем ссылку: https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json и нажимаем "ОК". 
После заходим в инструменты->плата->менеджер плат тут в строке поиска вводим esp32, выбираем соответсвующий вариант снизу и нажимаем на кнопку установить. 
После всех этих процедур для прошивки станут доступны платы семейства  ESP32.  Чтоб выбрать нужную плату заходим в инструменты->esp32 и выбираем нужную плату (мы использовали "ESP32 dev Module"). Также необходимо подключить саму плату к компьютеру и выбрать порт к которому она подключена заходим опять в инструменты->порт и выбираем нужный порт.
Помимо этого необходимо установить библиотеку для адресной светодиодной ленты. Для этого заходим в скетч->подключить библиотеку->Управлять библиотеками в поле поиска вводим "neopixel" и выбираем Adafruit neopixel и нажимаем установить.



Шаг 1. НАСТРОЙКА МОТОРОВ

Здесь нам нужно понять как подключены моторы и сделать так, чтоб робот ехал вперед, когда мы действительно этого хотим.

-проверить направление вращения моторов
-проверить подключение энкодеров

Скачиваем или копируем код для esp32 из папки low_level "test_move_esp32", открфываем этот код в arduino ide и нажимаем в правом верхнем углу "загрузить".

После полной загрузки кода колеса робота должны вращаться вперед, если это не так, то следует поменять в коде местами номера пинов, к которым подключён драйвер моторов левой или право стороны.
Для определения вращения энкодеров открываем в arduino IDE serial port и следим за значениями: оба значения на левом и правом моторах должны возрастать, если это не так, то меняем местами пины энкодера в программе.

Шаг 2. Загрузка основной программы на esp32.

Для этого шага нам необходимо скопировать все параметры из прдыдущего кода, а также измерить диаметр колеса и расстояние между центрами колёс.
Все эти параметры вашего робота необходимо внести в скетч main_esp32.ino, который находится в папке low-level. Этот код вы также скачиваете (или копируете) и открываете в Arduino IDE для дальнейшей прошивки. После внесения всех изменений в вашу программу по аналогии с 1 шагом загружаем программу на  микроконтроллер.

# III МОДУЛЬ - Настройка и программирование верхнего уровня

До запуска робота остался последний и самый ответственный шаг. Настройка микрокомпьютера - наших "мозгов" робота. В нашем проекте эту роль выполняла raspberry pi 5 8gb, поэтому советуем все повторять на таком же железе для предотвращения проблем. Безусловно, подойдет и другое железо, по типу repka pi, orange pi, banana pi и подобных "фруктов". Но наиболее популярной и поддерживаемой всетаки остается "малинка".

Шаг 1. Устанавливаем ubuntu.

По сути raspberry pi является аналогом вашего ноутбука или настольного компьютера, но куда меньших размеров и, скорее всего, хуже по вычислительным мощностям. Но тем не менее нам нужно поставить на наш микрокомпьютер операционную систему - наиболее подходящая ubuntu 24.04. !!! при сборке нашего робота мы сначала поставили 24.10 - по принципу: больше - лучше, но тут,увы, так не работает. И чтобы у вас без проблем все встало как нужно необходимо использовать именно ubuntu 24.04 LTS!!! 
Скачать ее можно с официального сайта - она абсолютно бесплатная. После загрузки образа диска необходимо его записать на sd карту, которая будет стоять в малинке. Мы использовали на 64гб. Или же можно сразу скачать Raspberry PI Imager(https://www.raspberrypi.com/software/) и там уже в интерфейсе выбрать нужную операционную систему и ваш носитель.

Шаг 2. Устанавливаем ROS2

После установки и базовых настроек операционной системы можно установить и сам ros2 jazzy (в нашем проекте используется десктопная версия ros-jazzy-desktop), лучше всего брать команды для установки с официального сайта (https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html), так как они могут меняться. 

Если установить не получается пробуем еще раз, внимательно копируем все и вставляем в терминал. Если совсем не помогает, то можно попытаться погуглить ошибку, которую вам выдает терминал или же переустановить заново операционку на флешку и убедится что это точно ubuntu 24.04 LTS.

Также установим еще пару пакетов:

```
sudo apt install python3-pip
```

```
pip3 install setuptools
```

```
sudo apt install python3-colcon-common-extensions
```

Шаг 3. Настраиваем нужные пакеты и едем!


После установки ros2 в домашнеq директории нужно создать папку где будут хранится все наши пакеты для ros2.

```
mkdir ros2_ws
```

```
cd ros2_ws
```

```
mkdir src
```

```
cd-
```

```
colcon build
```

после создания нашего рабочего пространства
создадим пакет, который будет считывать сообщения из топика /cmd_vel

```
cd ~/ros2_ws/src
ros2 pkg create robot_control --build-type ament_python --dependencies rclpy geometry_msgs
cd robot_control/robot_control
```

создаем файл robot_controller_node.py и копируем в него код из одноименного файла папки high_level. Предварительно также нужно подключить прошитую esp32 к одному из портов raspberry pi, а затем ввести команду: 
```
ls -l /dev/serial/by-path/
```

после необходимо скопировать путь и заменить его в коде.
сохраняем файл. Делаем скрипт исполняемым:
```
chmod +x robot_controller_node.py
```

Далее находим и редактируем файл setup.py в этом пакете (нужно добавить 3-ю строчку):

```
entry_points={
    'console_scripts': [
        'robot_controller_node = robot_control.robot_controller_node:main',
    ],
},
```

Затем пересоберем наш пакет и установим все зависимости:

```
cd ~/ros2_ws
colcon build
source install/setup.bash
```

И запускаем наш пакет:

```
ros2 run robot_control robot_controller_node
```

Проверяем список топиков в другом терминале, должен появиться топик /cmd_vel

```
source /opt/ros/jazzy/setup.bash
ros2 topic list
```

Если топик появился, то все прошло успешно и мы можем управлять нашим роботом с клавиатуры!

Для этого установим пакет:

```
sudo apt install ros-jazzy-teleop-twist-keyboard
```
И запускаем пакет: 
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Теперь робот должен ехать! Поздравляю мы проделали большой путь и запустили робота!
