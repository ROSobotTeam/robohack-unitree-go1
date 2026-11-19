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
