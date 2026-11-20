# robohack-unitree-go1 (T1 + T2)

Решение команды для заочного этапа Hackathon Go1 (задания T1 и T2 — блок безопасности низкоуровневого управления `Safety Block`).

Образ собран для `linux/amd64` на базе Ubuntu 22.04, ROS 2 Humble и Eclipse Cyclone DDS (`rmw_cyclonedds_cpp`), запускается от непривилегированного пользователя `participant` без доступа к интернету.

## Реализованные функции (T2)

Узел `safety_block/safety_node.py` подписывается на `/lowcmd_bare` и `/lowstate` и публикует проверенные команды в `/lowcmd` с частотой **100 Гц** по симуляционному времени (`use_sim_time:=true`):

* отклонение сообщения целиком при наличии `NaN`, `+Inf` или `-Inf` в управляющих полях (`0..11`);
* проверка угловых ограничений всех 12 суставов (`FR`, `FL`, `RR`, `RL`) и допустимых пределов `dq`, `kp`, `kd`, `tau`;
* ограничение изменения целевого угла между последовательными принятыми командами ($\le 5^\circ$);
* проверка самоколлизий в целевой позе и на переходах с дискретностью $0.1^\circ$ в MuJoCo 3.3.7 по 655 парам из `models/unitree_go1/t2_collision_pairs.csv` (глубина проникновения $\le 1\text{ мм}$);
* переход в безопасное удержание последней принятой целевой конфигурации (`mode = 10`, `dq = 0`, `tau = 0`) при потере входного потока более чем на 0,45 с симуляционного времени.

## Структура проекта

* `Dockerfile` — сборка образа `linux/amd64` с `unitree_go`, `mujoco==3.3.7` и пакетом `safety_block`
* `entrypoint.sh` — точка входа `/entrypoint.sh`, поддерживающая режимы `ping` и `T2`
* `ping.json` — метаданные протокола `ping` (`supported_tasks: ["T2"]`)
* `safety_block/` — ROS 2-пакет с узлом `safety_node`
* `models/unitree_go1/` — эталонная модель `go1.xml`, `scene.xml` и таблица `t2_collision_pairs.csv`

## Локальная сборка

```bash
docker buildx build --platform linux/amd64 --load -t rosobot-t2:dev .
```

## Проверка `ping` (без сети)

```bash
time timeout 10s docker run --rm --pull=never --network none \
    --entrypoint /entrypoint.sh rosobot-t2:dev ping
```

## Совместный запуск с публичным симулятором (`go1-simulator-public`)

```bash
python3 run_local.py --task T2 \
  --sim-image "$SIM_IMAGE" \
  --participant-image rosobot-t2:dev \
  --headless --logs
```
