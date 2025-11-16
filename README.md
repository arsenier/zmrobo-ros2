# Робот ZMRobo ROS2

На роботе настроен ROS2 Humble c использованием Docker

## Сборка и инициализация

```bash
make vizanti_build
make zmrobo_build
```

## Запуск робота и визуализации

```bash
make vizanti
```

```bash
make zmrobo_run
make zmrobo_into

cd workspace
# Драйвер лидара
make lidar

# Драйвер тележки
make zmrobo
```

