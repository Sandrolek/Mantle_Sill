# Mantle_Sill
Это репозиторий команды Мантия-Подоконник на соревнованиях ОКД НТИ 2021 по профилю Летательная Робототехника

Ссылка на папку с файлами записей rviz и папкой с файлами для gazebo simulator: https://drive.google.com/drive/u/0/folders/1CvDAdrmR2rgRnFcQhFUg2tGIh1ZDiCbq

Автор кода: Александр Поляков, https://vk.com/sandrolek

Общая логика третьей программы, как самой интересной и набитой разными аспектами программирования:
- взлетаем
- летим к qr и читаем его
- разделяем полученное сообщение на коорды препятствий, коорды стрелки и номер заказа
- летим к стрелке в считанные координаты
- применяем по очереди маски всех трех цветов к кадру со стрелкой, если ни одна маска не подошла, считаем что стрелка черная
- получили изображение mono8, т.е. черно-белое
- ищем контуры на полученном изображении, используя заранее обученную модель определяем тип стрелки (ее направление)
- летим в центр полученного сектора
- запускаем алгоритм посадки на центр наибольшего контура в маске цветка клевера
- перез самой посадкой включаем индикацию и садимся
- резко взлетаем, чтобы сразу прочитать aruco метки
- летим в 0;0
- садимся
- выводим общее время работы

Команды rosbag:
1 день: rosbag record /aruco_map/visualization /wp_markers /main_camera/image_raw /vehicle_marker /rangefinder/range
2 день: rosbag record /aruco_map/visualization /wp_markers /main_camera/image_raw /vehicle_marker /rangefinder/range /Detect
3 день: rosbag record /aruco_map/visualization /wp_markers /main_camera/image_raw /vehicle_marker /rangefinder/range /Detect /aruco_map/pose
