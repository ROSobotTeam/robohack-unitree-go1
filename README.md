# robohack-unitree-go1

## T2 — Safety Block

Узел проверяет команды из `/lowcmd_bare` и публикует безопасные команды в `/lowcmd`.
При потере входа удерживает последнюю безопасную цель.

Сборка из корня репозитория команды:

```bash
docker buildx build --platform linux/amd64 --load -t rosobot-t2:dev .
```

Проверка образа без сети, из того же каталога:

```bash
docker run --rm --network none rosobot-t2:dev ping
```

## T3 — Ходьба с RL-политикой

В режиме `T3` узел политики читает `/cmd_vel` и `/lowstate`, вычисляет 12 целевых углов на CPU и публикует их в `/lowcmd_bare`. Safety Block проверяет переход и публикует безопасную команду в `/lowcmd`. Режим `T2` запускает Safety Block отдельно. В `ping.json` указана скорость 0,3 м/с.

### Сборка и проверка

В WSL из корня репозитория команды собери образ и проверь `ping`.

```bash
docker buildx build --builder default --provenance=false --platform linux/amd64 --load -t rosobot-t3:release-candidate .
timeout 10s docker run --rm --pull=never --network none rosobot-t3:release-candidate ping
```

В первом терминале WSL из корня клона симулятора `go1-simulator-public` запусти пару контейнеров. Дождись `Pair started` и оставь терминал работающим.

```bash
python3 run_local.py --task T3 --headless --logs \
  --sim-image registry.cookcloud.ru/go1-hackathon-remote-stage/go1-sim:v1.0 \
  --participant-image rosobot-t3:release-candidate
```

Во втором терминале WSL из корня исходников T2 и T3 выполни `check_t3.py` внутри контейнера Simulator. Введи имя из актуальной строки `Simulator=`. В контейнере скрипт выполняется в `/home/ros`.

```bash
read -r -p 'Имя контейнера из строки Simulator=: ' SIM_CONTAINER
docker exec -i --workdir /home/ros "$SIM_CONTAINER" bash -c \
  'source /opt/ros/humble/setup.bash && source /home/ros/go1_ws/install/setup.bash && source /etc/go1_sim_env.sh && exec python3 -' \
  < check_t3.py
```

После отчёта `check_t3.py` останови первый терминал через `Ctrl+C`, чтобы завершить оба контейнера.

### Подтверждение обучения

Участник ROSobot выполнил обучение PPO в запуске `20261124_104638`. Экспорт `policy.npz` совпадает с финальным чекпоинтом на 504 627 200 шагах среды. SHA-256 весов — `c44333ad59b4205e36e1d8d73864c28e1178c607ae16063a1d7589339e9ab3e3`.

- [T3_go1_rl_training_10.ipynb](training/T3_go1_rl_training_10.ipynb) содержит код обучения PPO, экспорт весов и сохранённые выводы запуска `20261124_104638` со списком чекпоинтов.
- `training/config.json` содержит конфигурацию обучения.
- `training/metrics.csv` и `training/reward_curve.png` содержат журнал и график обучения.
- `training/sim2sim_results.csv` и `training/sim2sim_summary.json` содержат результаты симуляции с экспортированной политикой.
- `training/t3-series-20261124.json` содержит отдельный локальный отчёт об интеграции через Safety Block. Диагностическая сборка прошла 9 из 10 эпизодов на 0,3 м/с.

В проверке `sim2sim` ноутбук применяет моменты напрямую в MuJoCo и моделирует фиксированные задержки 0 и 20 мс. В локальном отчёте проверен запуск через ROS 2 и Safety Block.
