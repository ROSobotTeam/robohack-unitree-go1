# robohack-unitree-go1

## T1 — Docker-образ участника

Образ для `linux/amd64` на базе Ubuntu 22.04 с ROS 2 Humble и CycloneDDS.

### Локальная сборка

```bash
docker buildx build --platform linux/amd64 --load -t rosobot-t1:dev .
```

### Проверка `ping`

```bash
time timeout 10s docker run --rm --pull=never --network none \
    --entrypoint /entrypoint.sh rosobot-t1:dev ping
printf 'exit=%s\n' "$?"
```

Ожидается один JSON-объект, код выхода `0` и время выполнения до 10 секунд.
