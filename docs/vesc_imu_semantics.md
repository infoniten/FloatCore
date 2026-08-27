# Семантика IMU в VESC: что именно отдаёт `VESC_IF`

Документ отвечает на один вопрос: **в каких единицах и в какой системе
координат прошивка VESC отдаёт данные IMU пакетам вроде Refloat.** Всё
доказывается по исходникам `vedderb/bldc`, а не по поведению.

До v0.6D.1 в проекте было два незакрытых места: единицы `imu_get_gyro()` были
выведены косвенно, из масштабов PID, и не был известен слой, отвечающий за
ориентацию. Оба закрыты.

---

## 1. Полная цепочка данных

Восстановлена по `bldc/imu/imu.c`, `bldc/imu/ahrs.c` и
`bldc/lispBM/lispif_c_lib.c`.

```
драйвер датчика
   accel[3] в g, gyro[3] в °/с, mag[3]
        ↓
   поворот матрицей m11…m33, построенной из
   m_settings.rot_roll / rot_pitch / rot_yaw   ← ОРИЕНТАЦИЯ И МОНТАЖ
        ↓
   вычитание m_settings.accel_offsets / gyro_offsets  ← СМЕЩЕНИЯ ДАТЧИКА
        ↓
   биквадратные ФНЧ (accel_lowpass_filter_*, gyro_lowpass_filter)
        ↓
   m_accel[3] в g,  m_gyro[3] в °/с
        ├──────────────────────────────► imu_get_accel()  → g
        ├──────────────────────────────► imu_get_gyro()   → °/с
        ↓
   gyro_rad[i] = DEG2RAD_f(m_gyro[i])
        ↓
   ahrs_update_mahony_imu(gyro_rad, m_accel, dt, &m_att)
        ├──────────────────────────────► imu_get_roll/pitch/yaw() → радианы
        ↓
   m_read_callback(m_accel, gyro_rad, m_mag, dt)
                                        → acc в g, gyro в РАД/С
```

### Порядок операций доказан

`bldc/imu/imu.c`, внутри callback драйвера датчика, именно в этом порядке:

```c
m_accel[0] = accel[0] * m11 + accel[1] * m12 + accel[2] * m13;
...
m_gyro[0]  = gyro[0]  * m11 + gyro[1]  * m12 + gyro[2]  * m13;
...
for (int i = 0; i < 3; i++) {
    m_accel[i] -= m_settings.accel_offsets[i];
    m_gyro[i]  -= m_settings.gyro_offsets[i];
}
```

**Смещения вычитаются ПОСЛЕ поворота**, то есть хранятся в уже повёрнутой
системе координат. Это не деталь оформления: если реализовать наоборот,
смещения будут вычитаться не из тех осей.

---

## 2. `imu_get_gyro()` — градусы в секунду

```
imu_get_gyro() returns:
[ ] rad/s
[x] deg/s
```

Доказательство прямое и не опирается на масштабы PID.

**Шаг 1.** `bldc/imu/imu.c`:

```c
void imu_get_gyro(float *gyro) {
	memcpy(gyro, m_gyro, sizeof(m_gyro));
}
```

Функция отдаёт `m_gyro` без каких-либо преобразований.

**Шаг 2.** В том же файле, в callback драйвера:

```c
float gyro_rad[3];
gyro_rad[0] = DEG2RAD_f(m_gyro[0]);
gyro_rad[1] = DEG2RAD_f(m_gyro[1]);
gyro_rad[2] = DEG2RAD_f(m_gyro[2]);
```

`m_gyro` **конвертируется из градусов в радианы** перед подачей в AHRS.
Конвертировать из градусов можно только то, что в градусах. Следовательно
`m_gyro`, а с ним и `imu_get_gyro()`, — °/с.

**Шаг 3.** `bldc/lispBM/lispif_c_lib.c` связывает интерфейс пакетов напрямую:

```c
cif.cif.imu_get_gyro = imu_get_gyro;
```

Никакой обёртки с пересчётом между `VESC_IF->imu_get_gyro` и функцией прошивки
нет.

---

## 3. Массив `gyro` в callback — радианы в секунду

Та же функция, последняя строка:

```c
if (m_read_callback) {
    m_read_callback(m_accel, gyro_rad, m_mag, dt);
}
```

Передаётся **`gyro_rad`**, а не `m_gyro`. Это и есть callback, который
регистрирует Refloat через `imu_set_read_callback`.

Подтверждается и со стороны потребителя: `bldc/imu/ahrs.c`,
`ahrs_update_mahony_imu()` берёт `gyroXYZ` и интегрирует кватернион без единого
преобразования градусов — та же математика, что в `balance_filter.c` Refloat.

**Итог: две функции VESC_IF отдают гироскоп в РАЗНЫХ единицах.** Это не
ошибка API и не описка: `imu_get_gyro()` — «человеческий» интерфейс в °/с,
callback — вход фильтра ориентации в рад/с.

---

## 4. Единицы остальных функций

| Функция `VESC_IF` | Единицы | Доказательство |
|---|---|---|
| `imu_get_accel()` | g | `m_accel` не масштабируется; `ahrs.c` сравнивает норму с единицей |
| `imu_get_gyro()` | **°/с** | §2 |
| `imu_get_roll/pitch/yaw()` | **радианы** | `ahrs_get_*()` возвращают результат `atan2f`/`asinf` без пересчёта |
| `imu_get_rpy()` | радианы | те же функции |
| `imu_set_yaw(float yaw_deg)` | **градусы** | имя параметра в `imu.h` |
| callback `acc` | g | §3 |
| callback `gyro` | **рад/с** | §3 |
| callback `dt` | секунды | подаётся в AHRS как `dt` |

---

## 5. Соглашение об углах

`bldc/imu/ahrs.c`:

```c
float ahrs_get_roll(const ATTITUDE_INFO *att) {
	return -atan2f(q0 * q1 + q2 * q3, 0.5 - (q1 * q1 + q2 * q2));
}
float ahrs_get_pitch(const ATTITUDE_INFO *att) {
	return asinf(-2.0 * (q1 * q3 - q0 * q2));
}
float ahrs_get_yaw(const ATTITUDE_INFO *att) {
	return -atan2f(q0 * q3 + q1 * q2, 0.5 - (q2 * q2 + q3 * q3));
}
```

Это **посимвольно те же формулы**, что в `refloat-upstream/src/balance_filter.c`
(:136-164), включая ведущие минусы в roll и yaw. Refloat скопировал AHRS
прошивки и изменил только назначение фильтра, а не соглашение о знаках.

Практическое следствие для FloatCore: платформенный AHRS
(`compat/imu/fc_ahrs.c`) реализован по этим же формулам, а значит его знаки
совпадают и с прошивкой VESC, и с `balance_filter` Refloat по построению.

---

## 6. Где именно Refloat пользуется гироскопом

Полный перечень, `refloat-upstream/src`:

| Место | Компонента | Ожидаемые единицы | Чем подтверждается |
|---|---|---|---|
| `main.c:742` → `balance_filter_update(gyro, acc, dt)` | все три | **рад/с** | `balance_filter.c:115-124` интегрирует `q += ½·q·ω·dt` |
| `imu.c:44` `VESC_IF->imu_get_gyro(gyro)` | — | **°/с** | §2 |
| `imu.c:51` `pitch_rate = cos²(roll)·gyro[1] + sin·cos·gyro[2]` | Y и Z | °/с | тот же массив |
| `pid.c:69` `rate_p = -pitch_rate · kp2 · TORQUE_CONSTANT_COMPAT` | — | °/с | единственный потребитель `pitch_rate` |

**Обращение к `imu_get_gyro()` в Refloat ровно одно.** Его результат уходит
только в демпфирующий член PID и никуда больше — ни в телеметрию, ни в
фильтр ориентации.

Дополнительно `turn_tilt.c:63,70,73` работает с `imu->yaw` (градусы после
`rad2deg`) как с **приращением**, ограничивая `new_change / dt` величиной
±72 — тоже градусы в секунду.

### Проверка текущей реализации FloatCore

| Что | VESC | FloatCore | Совпадает |
|---|---|---|---|
| callback `gyro` | рад/с | рад/с (`fc_imu_source.c`, `s.gyro_rad_s`) | да |
| `imu_get_gyro()` | °/с | °/с (`s.gyro_dps`) | да |
| callback `acc` | g | g | да |
| `imu_get_roll/pitch/yaw()` | радианы | радианы | да |
| формулы углов | `ahrs.c:ahrs_get_*` | те же (`fc_ahrs.c`) | да |

**Ошибки единиц нет.** Косвенный вывод из масштабов PID, сделанный на v0.6D,
подтвердился прямым доказательством по исходникам. Менять ничего не нужно.

---

## 7. Что из этого следует для контракта FloatCore

`docs/imu_contract.md` дополнен ссылками на этот документ. Существенное:

* две функции гироскопа имеют разные единицы — это норма, а не ошибка;
* повороты осей и вычитание смещений выполняются **до** всего остального, то
  есть и AHRS, и callback получают уже исправленные данные;
* порядок «сначала поворот, потом смещения» обязателен к воспроизведению.
