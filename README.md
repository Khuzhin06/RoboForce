В данном курсе мы расскажем, как разработать робота на ROS2 с ESP32 Raspberry Pi 5.

Благодарим организаторов "ROS 2 - Народный курс" за предоставленные учебные материалы и технические ресурсы.
Ознакомиться с ними вы можете по ссылке: https://stepik.org/course/221157/syllabus

Также много интересного и полезного можете найти в нашем TG-канале https://t.me/RoboForc 

I модуль - сборка робота
1. Подготовка материалов и инструментов
Необходимые инструменты:
Лазерный резак/лобзик – для резки фанеры.
Дрель и сверла (2–3) – для отверстий.  
Набор отверток с разными насадками 
Паяльник (40 Вт) + припой (POS-60) – для электроники.  
Мультиметр – проверка цепей.  
Наждачка – обработка краёв

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

2. Изготовление корпуса

Шаг 1: Резка деталей
Загрузите DXF-файлы в станок лазерной резки.  
Основные детали:  
Основание (нижняя платформа)
Средняя платформа
Крышка (верхняя платформа)
2 опоры для OMNI колеса
5 стоек по 60 мм
4 стойки по 130 мм
После резки обработайте края наждачкой, чтобы избежать заусенцев.  

Шаг 2: 3D-печать
Загрузите DXF-файлы в станок 3D-печати.  
Основные детали:  
2 колеса
Стойки для моторчиков
![image](https://github.com/user-attachments/assets/6dffb7dc-7430-4fb7-bbe7-f751f1ad7de7)
![image](https://github.com/user-attachments/assets/e265e2c7-af5c-46d4-87a9-87122f94bcaa)

Шаг 3: Сборка каркаса (первый слой)
Установка мотор редукторов
На валы мотор-редукторов установите колёса, закрепите их стопорными винтами.
![image](https://github.com/user-attachments/assets/9f04b441-88c4-4640-9b63-5f7a0e0fc872)

Найдите на нижней платформе посадочные места под моторы.
Зафиксируйте каждый мотор винтами M3 х 50 мм (4 шт) и гаек М3 (4 шт) с помощью напечатанных стоек (обратите внимание на то, чтобы провода от энкодеров были выведены к центру платформы)
Убедитесь, что при вращении колёса не задевают за край платформы и установлены ровно.
Уделите внимание ровной установке колес, это обеспечит качественное движение без лишних корректировок.
![image](https://github.com/user-attachments/assets/dd758873-1871-4d8a-adda-b4ce5c991556)
![image](https://github.com/user-attachments/assets/e09da91f-3ad2-423f-b8ae-80c137f1c8ae)

Установка OMNI колеса 
Найдите на нижней платформе посадочные места под вырезанные опоры OMNI колеса.
Установите их с помощью винтов М3 х 20 (4 шт) и гаек М3 (4 шт)
![image](https://github.com/user-attachments/assets/01de58ea-3ed2-43d3-945e-6c7934180a54)
![image](https://github.com/user-attachments/assets/226cf342-804d-4183-8706-759e79d707e0)

В отверстие опоры вставьте винт М3 х 50, немного накрутите на него гайку М3
![image](https://github.com/user-attachments/assets/57e0ae9f-9023-42b4-9d1f-cbbb0eee13f3)

Установите OMNI колесо на винт и закрутите (с внутренней стороны колеса) гайками М3 (2шт)
![image](https://github.com/user-attachments/assets/dcc521c2-1069-4f9c-9af2-20bf28b7fd74)
![image](https://github.com/user-attachments/assets/5e42c6bb-d48d-4c51-aebf-f17f88c949b8)

С внешней стороны опоры OMNI колеса зафиксируйте винт гайкой М3
Убедитесь, что при вращении колёсо не задевает за край платформы.
Установка драйвера моторов
Найдите в центре нижней платформы посадочные места под драйвер
Установите драйвер с помощью винтов М3 (2шт) и гаек М3 (2шт)
![image](https://github.com/user-attachments/assets/9039d366-f1d5-4f64-aa69-340d8d99fd79)

Шаг 4: Сборка каркаса (средний слой)
Пайка шилда для ESP
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
1. Визуальный осмотр:  
   - Нет ли перемычек между контактами (проверка на отсутствие короткого замыкания)
 Все ли дорожки пропаяны (особенно у штыревых разъёмов).  
2.  Тест мультиметром:  
   - Прозвоните линии 5V и 3.3V на КЗ.  
   - Проверьте контакты GPIO на обрыв.  
Установка шилда на платформу
Найдите на средней платформе посадочное место для шилда
Вставьте снизу в платформу винты М2 (4шт) и накрутите на них гайки М2 (4шт). Данная конструкция не позволит передать находящиеся под шилдом провода  
Установите на винты собранный шилд и закрепите его сверху гайками М2 (4шт)
![image](https://github.com/user-attachments/assets/030c5e43-1274-4afe-9a40-431a16b0ff15)

Установка блока питания 
Найдите на средней платформе посадочное место для блока питания (средняя часть платформы) 
Закрепите блок с помощью винтов М3 (2 шт) и гаек М3 (2шт)  (ВАЖНО: провода должны выходить в центральную часть платформы)
![image](https://github.com/user-attachments/assets/b63c3e8f-8db2-4674-855f-9d3161c8c713)
![image](https://github.com/user-attachments/assets/0fa06e61-8c34-468a-b8a7-fc7e63bf7e52)


Монтаж: 
Схему соединений см. в "electronic scheme"

Пайка проводов
- Залудите контакты перед пайкой, а затем работайте по тому же принципу, как и при пайке шилда  
После окончания пайки необходимо провести проверку (визуальный осмотр и тест мультиметром) (ВАЖНО: проверьте, нет ли пережатых проводов)
![image](https://github.com/user-attachments/assets/de35dbd0-492c-492b-9d6c-5eb8d78bf625)
![image](https://github.com/user-attachments/assets/2defbdcc-f68b-4a34-892a-fc94218b6f51)
![image](https://github.com/user-attachments/assets/f270cfb1-d21b-4bdc-842e-68b68ae3c9c5)

Шаг 3: Подсоединение нижней и верхней платформы:
Соберите стойки из фанеры: 5 штук по 60 мм, 4 штуки по 130 мм (для этого понадобятся гайки М3 18 шт)
Соедините нижнюю и верхнюю платформы с помощью стоек по 60 мм, винтов М3 (10шт)
![image](https://github.com/user-attachments/assets/9d75c9f5-c1b5-4cfd-8cf2-11af3c26be24)
![image](https://github.com/user-attachments/assets/ef5b23e3-8283-4068-adc9-62259d38eddd)
![image](https://github.com/user-attachments/assets/f2c281f6-5179-434e-a022-b95a987e2805)
![image](https://github.com/user-attachments/assets/67f2db67-e9ac-4b81-8ecf-74aa50b34ac0)


Шаг 5: Тестовый запуск
Подключите питание. 
Проверьте работу моторов, питание и загрузку ESP.
![image](https://github.com/user-attachments/assets/0a724450-055a-4aee-9a32-913ad60914eb)
![image](https://github.com/user-attachments/assets/680014d7-5151-4f43-a8e2-11e9fd9dd73b)
![Uploading image.png…]()


Модуль II - программироварние ESP32

После того как вся аппаратная часть собрана, можно переходить и к программной.
Все действия лучше делать на ноутбуке - так как это будет быстрее и надежней, хотя в теории можете и сделать все тоже самое на raspberry.
Начнем с программирования "нижнего уровня", тоесть esp32. Программировать будем через arduino IDE (если она еще у вас не установлена, то можете скачать с официального сайта https://www.arduino.cc/en/software ).
Чтоб можно было прошивать нашe платe добавим ее в список доступных для прошивки. Нужно зайти в файл->настройки и в поле "Дополнительные менеджеры плат" вставляем ссылку: https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json и нажимаем "ОК". 
После заходим в инструменты->плата->менеджер плат тут в строке поиска вводим esp32, выбираем соответсвующий вариант снизу и нажимаем на кнопку установить. 
После всех этих процедур для прошивки станут доступны платы семейства  ESP32.  Чтоб выбрать нужную плату заходим в инструменты->esp32 и выбираем нужную плату (мы использовали "ESP32 dev Module"). Также необходимо подключить саму плату к компьютеру и выбрать порт к которому она подключена заходим опять в инструменты->порт и выбираем нужный порт.
Помимо этого необходимо установить библиотеку для адресной светодиодной ленты. Для этого заходим в скетч->подключить библиотеку->Управлять библиотеками в поле поиска вводим "neopixel" и выбираем Adafruit neopixel и нажимаем установить.


Шаг 1. Настройка моторов

Здесь нам нужно понять как подключены моторы и сделать так, чтоб робот ехал вперед, когда мы действительно этого хотим.

-проверить направление вращения моторов
-проверить подключение энкодеров

Скачиваем или копируем код для esp32 из папки low_level "test_move_esp32", открфываем этот код в arduino ide и нажимаем в правом верхнем углу "загрузить".

После полной загрузки кода колеса робота должны вращаться вперед, если это не так, то следует поменять в коде местами номера пинов, к которым подключён драйвер моторов левой или право стороны.
Для определения вращения энкодеров открываем в arduino IDE serial port и следим за значениями: оба значения на левом и правом моторах должны возрастать, если это не так, то меняем местами пины энкодера в программе.


Стоимость компонентов для сборки робота:

За относительно небольшой бюджет можно также собрать достаточно надежного и производительного робота.
Суммируя все цены комплектующих из I модуля 1 пункта, мы получаем итоговую стоимость нашей модели - 22.600.
