В данном курсе мы расскажем, как разработать робота на ROS2 с ESP32 Raspberry Pi 5

I модуль - сборка робота
1. Подготовка материалов и инструментов
Необходимые инструменты:
- Лазерный резак/лобзик – для резки фанеры.
- Дрель и сверла (2–3) – для отверстий.  
- Паяльник (40 Вт) + припой (POS-60) – для электроники.  
- Мультиметр – проверка цепей.  
- Наждачка – обработка краёв.  

2. Материалы:

Компонент
Кол-во![photo_5444974429529565123_y](https://github.com/user-attachments/assets/8af5a3d7-5805-4eac-9df8-87dfa37b2a5b)

Примечание
Фанера 6 мм
1 лист
600 х 600
Винты М3
33
30х20 мм, 3х50мм
Гайки М3
21
3 из них для закрепления OMNI колеса
Винты М2
4
10мм. Для закрепления шилда с ESP
Гайки М2
4


2. Изготовление корпуса
Шаг 1: Резка деталей
Загрузите DXF-файлы в станок лазерной резки.  
Основные детали:  
Основание (нижняя плита)
Средняя плита
Крышка (верхняя плита)
2 опоры для OMNI колеса
5 стоек по 60 мм
4 стойки по 130 мм
После резки обработайте края наждачкой, чтобы избежать заусенцев.  

Шаг 2: 3D-печать
Загрузите DXF-файлы в станок 3D-печати.  
Основные детали:  
2 колеса
Стойки для моторчиков

Шаг 3: Сборка каркаса
Монтаж колёс и опорных роликов
На валы мотор-редукторов установите колёса, закрепите их стопорным винтом.
Прикрутите дополнительное опорное колесо к задней или передней части платформы.
Убедитесь, что при вращении колёса не задевают за край платформы и установлены ровно.
Установка мотор-редукторов
Найдите на нижней платформе посадочные места под моторы.
Зафиксируйте каждый мотор винтами M3.
Уделите внимание ровной установке колес, это обеспечит качественное движение без лишних корректировок.
Оставшееся место - для аккумуляторов
Часто на нижней платформе размещают аккумуляторную батарею для того, чтобы центр тяжести робота был как можно ниже.
Соберите стойки из фанеры, закрепите их винтами M3.

3. Установка электроники
Комплектующие:

Список покупных компонентов таблицей
Название
Назначение/описание
Ссылка
Кол-во 
Фотографии 
Raspberry pi 5 8gb
«Верхний уровень». Обрабатывает информацию, поступающую с ESP (“нижнего уровня»). Поддержка новейших интерфейсов. Отлично подходит к задачам компьютерного зрения, обработки трёхмерных данных с лидаров. 
https://sl.aliexpress.ru/p?key=0vxkGbg
1
9249
ESP32-WROOM-32
«Нижний уровень». Собирает информацию с датчиков, энкодеров и отправляет на «верхний уровень». Подходит для сетевого проекта из-за встроенных Wi-Fi и Bluetooth. Плюсы: Высокая производительность, Wi-Fi/BT на борту, большой объём памяти, 3,3 В логика.
Минусы: Нужно продуманное питание 3,3 В, сложнее освоить низкоуровневые функции, GPIO имеют ограничения.
https://sl.aliexpress.ru/p?key=7oxkGZr
1
336
JGA25-370 77 RPM 12 B
Необходимы для передвижения робота. Используем 12В для упрощения питания.
https://aliexpress.ru/item/1005007793208347.html?sku_id=12000042220805718&spm=a2g2x.productlist.search_results.1.2ccd33e3p7kOox
2
2 x 610 руб
Драйвер моторов L298N
Позволяет обеспечить требуемую мощность и стабильность, достичь высокой энергоэффективности, надёжности и удобства интеграции в систему. Преимущества: доступность, использование 5В выхода для питания микроконтроллера, питание моторов до 30В, управление 2  DC моторами.
Недостатки: низкий КПД, сильный нагрев, нужно ставить радиатор.
https://aliexpress.ru/item/1005007125159518.html?sku_id=12000042919988210&srcSns=sns_More&businessType=ProductDetail&spreadType=socialShare&tt=MG&utm_medium=sharing 
1
154 руб
DC-DC преобразователи Readytosky FPV RC UBEC 5V 5A (
Обеспечивает стабильное питание для микроконтроллера, микрокомпьютера, датчиков и драйверов.
https://aliexpress.ru/item/33019505937.html?sku_id=67198302173&srcSns=sns_More&businessType=ProductDetail&spreadType=socialShare&tt=MG&utm_medium=sharing
2
2 x 410
Литиевые батареи JOUYM 18650, 2000 мАч
Обеспечивает питание робота. Преимущества: Высокая плотность энергии при относительно небольшом размере.
Удобство конфигурации: можно последовательно или параллельно соединять несколько штук, добиваясь нужного напряжения и ёмкости.
https://sl.aliexpress.ru/p?key=ajxkGkw
3
3 х 154
Лидар SLAMTEC RPLIDAR C1
«Глаза» робота, который смотрит на окружение, вычисляет расстояние, определяет препятствия впереди
https://sl.aliexpress.ru/p?key=fmxkGwR
1
7174

Шаг 4: Монтаж
.
.
Проверка: Мультиметром убедитесь, что нет короткого замыкания между .
и .
4. Пайка и подключение

Схема соединений:

Пайка проводов:
  - Залудите контакты перед пайкой, используя 
  - Не перегревайте компоненты (не держите паяльник дольше 3–5 сек).  

Шаг 5: Тестовый запуск
1. Подключите питание.  

Шаг 6. Финишная сборка
1. Закрепите крышку.
2. Проденьте провода через отверстия.  
3. Проверьте:  
   - Нет ли пережатых проводов.  
   - Все ли датчики доступны для работы. 

Шаг 7. Техника безопасности
- При пайке: Работайте в проветриваемом помещении, используйте очки.  
- При резке: Надевайте маску от пыли.  
- При первом запуске: Держите робота на весу, чтобы колёса/моторы не повредили поверхность.  

Список покупок:
