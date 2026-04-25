# Feedback-only SRV_FB path — توثيق التنفيذ

**الفرع:** `devin/1776986012-feedback-only-srvfb`
**التاريخ:** 2026-04-23
**المؤلف:** تنفيذ آلي (Devin) بناءً على خطة مُعتمَدة + ملاحظات مهندس برمجي مقبولة 8/8.

---

## 1. المشكلة الأصلية

RocketMPC كان يشتق `_de_act/_dr_act/_da_act` (زاوية الزعنفة الفعلية) عبر
first-order-lag filter داخلي على الأوامر (tau=15ms)، بينما CAN feedback الحقيقي
(من xqpower_can عبر SRV_FB) كان متاحاً في `debug_array` id=1 ومُهدراً داخل
RocketMPC.

النتيجة:
- **MHE** يستهلك `mhe_p[0..2] = {_de_act, _dr_act, _da_act}` → نموذج أيرو مضطرب.
- **MPC** يبدأ من `x_mpc[15..17] = {_de_act, _dr_act, _da_act}` → warm-start خاطئ.
- تحت max-Q: alpha bias ~1–3° + gyro_bias drift تراكمي.

المشكلة تظهر في 22004 (HITL) و 22005 (Real) على السواء.

## 2. الحل المعتمد

**Feedback-only** عند `ROCKET_USE_SRV_FB=1`:
- المصدر الوحيد = CAN SRV_FB (data[4..7] بالدرجات).
- عند انقطاع CAN → Hold آخر قيمة (لا معادلة).
- قبل أول SRV_FB → `_de_act = 0` (السيرفو في trim).

**حماية SITL** عند `ROCKET_USE_SRV_FB=0` (الوضع الافتراضي):
- الاحتفاظ بـ first-order-lag filter كما هو (SITL لا يوجد فيه CAN).

**لا مساس** بـ `MpcController::_forward_guess`، ولا بـ acados OCP، ولا بأي
كود مُولَّد من `m130_ocp_setup.py`. لا إعادة توليد للـ solver.

## 3. الملفات المُعدَّلة (8 ملفات)

| # | الملف | التعديل |
|---|---|---|
| 1 | `AndroidApp/app/src/main/cpp/PX4-Autopilot/msg/RocketGncStatus.msg` | إضافة 4 حقول: `can_stale_us`, `can_abort_events`, `first_fb_received`, `valid_mask` |
| 2 | `AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/rocket_mpc_params.c` | إضافة باراميتر `ROCKET_USE_SRV_FB` (INT32، default=0، min=0، max=1) |
| 3 | `AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/RocketMPC.hpp` | state vars + ثوابت + param ref |
| 4 | `AndroidApp/app/src/main/cpp/PX4-Autopilot/src/drivers/xqpower_can/XqpowerCan.cpp` | حقن per-servo age بالـ ms في `SRV_FB.data[14..17]` |
| 5 | `AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/RocketMPC.cpp` | استبدال كتلة تحديث `_de_act/_dr_act/_da_act` + reset + telemetry |
| 6 | `AndroidApp/.../ROMFS/px4fmu_common/init.d/airframes/22004_m130_rocket_mpc_hitl` | `param set ROCKET_USE_SRV_FB 1` |
| 7 | `AndroidApp/.../ROMFS/px4fmu_common/init.d/airframes/22005_m130_rocket_mpc_real` | `param set ROCKET_USE_SRV_FB 1` |
| 8 | `FEEDBACK_ONLY_SRVFB.md` (هذا الملف) | توثيق |

## 4. تفاصيل التعديلات

### 4.1 `RocketGncStatus.msg`

```
uint32 can_stale_us       # [us] Time since last valid SRV_FB frame
uint32 can_abort_events   # Count of entries into CAN-lost abort state (>500ms)
uint8  first_fb_received  # 1 after the first valid SRV_FB frame consumed
uint8  valid_mask         # Bitmask of servos used in the last back-solve
```

- عند `ROCKET_USE_SRV_FB=0` (SITL) تبقى كل الحقول صفراً.
- `valid_mask`:
  - `0x0F` = كل السيرفوهات الأربعة طازجة (full fresh back-solve).
  - `0x0E/0x0D/0x0B/0x07` = 3 طازجة + 1 مُستبدَل بآخر قيمة معروفة.
  - `0x00` = Hold كامل (≥2 مفقودة أو لا إطار صالح).

### 4.2 `rocket_mpc_params.c`

```c
PARAM_DEFINE_INT32(ROCKET_USE_SRV_FB, 0);
```

- Default = 0 → SITL آمن، يحتفظ بالسلوك القديم.
- تعليق توثيقي يشرح 0 مقابل 1 + الـ airframes التي تُفعّله.

### 4.3 `RocketMPC.hpp` (إضافات)

```cpp
// ── CAN feedback path state ──
float       _last_fb_per_servo_rad[4] {};
bool        _last_fb_per_servo_valid[4] {};
hrt_abstime _last_fb_time{0};
bool        _first_fb_received{false};
uint32_t    _can_stale_us{0};
uint32_t    _can_abort_events{0};
bool        _can_abort_warned{false};
uint8_t     _can_valid_mask{0};

static constexpr hrt_abstime CAN_FRESH_US    = 120_ms;
static constexpr hrt_abstime CAN_ABORT_US    = 500_ms;
static constexpr float       MAX_FB_RATE_DPS = 400.0f;

// في كتلة DEFINE_PARAMETERS
(ParamInt<px4::params::ROCKET_USE_SRV_FB>) _param_use_srv_fb
```

### 4.4 `XqpowerCan.cpp`

حقن per-servo age داخل `SRV_FB.data[14..17]` (ms منذ آخر CAN frame لكل سيرفو):

```cpp
for (int i = 0; i < XQPOWER_MAX_SERVOS; i++) {
    uint32_t age_ms = 0;
    if (_feedback[i].last_update_us != 0
        && fb_now >= _feedback[i].last_update_us) {
        uint64_t age_us = fb_now - _feedback[i].last_update_us;
        uint64_t age_ms_u64 = age_us / 1000ULL;
        if (age_ms_u64 > 65535ULL) age_ms_u64 = 65535ULL;
        age_ms = (uint32_t)age_ms_u64;
    } else {
        age_ms = 65535u;  // no frame yet → max age
    }
    dbg.data[14 + i] = (float)age_ms;
}
```

- `data[12]` (online_mask) و `data[13]` (tx_fail_count) **لم تُلمس**، للحفاظ
  على توافق consumers الموجودين.

### 4.5 `RocketMPC.cpp`

**أ) Reset في `_reset_flight_state`:**
```cpp
for (int i = 0; i < 4; ++i) {
    _last_fb_per_servo_rad[i] = 0.0f;
    _last_fb_per_servo_valid[i] = false;
}
_last_fb_time = 0;
_first_fb_received = false;
_can_stale_us = 0;
_can_abort_events = 0;
_can_abort_warned = false;
_can_valid_mask = 0;
```

**ب) استبدال كتلة تحديث `_de_act/_dr_act/_da_act`** (السابقة في ~981-994):

```cpp
if (_param_use_srv_fb.get() == 0) {
    // ── Legacy first-order lag (SITL) ──
    float tau = _mpc.config().tau_servo;
    if (tau < 1e-4f) tau = 1e-4f;
    float decay = expf(-dt / tau);
    _de_act = _de_act * decay + delta_e * (1.0f - decay);
    _dr_act = _dr_act * decay + delta_r * (1.0f - decay);
    _da_act = _da_act * decay + delta_a * (1.0f - decay);
} else {
    // ── CAN SRV_FB feedback-only path (HITL / real) ──
    // 1. copy SRV_FB + reject if id ≠ 1, name ≠ "SRV_FB", or (now-ts) ≥ 120ms
    // 2. for each servo i ∈ {0..3}: if online && per-servo age ≤ 120ms →
    //    _last_fb_per_servo_rad[i] = data[4+i] * deg2rad, fresh_mask |= (1<<i)
    // 3. popcount(fresh_mask):
    //    == 4 → back-solve from 4 fresh;   valid_mask = 0x0F
    //    == 3 → استبدال المفقود بآخر قيمة معروفة (إن وُجدت) → back-solve
    //    ≤ 2 → hold (de,dr,da) ككل؛       valid_mask = 0x00
    // 4. rate-limit: if !first_fb || all |new - old| < 400°/s * dt → apply
    //    otherwise hold لدورة واحدة (CAN spike rejection)
    // 5. on no update: tick can_stale_us; if > 500ms → mavlink_log_critical +
    //    can_abort_events++ (one-shot via _can_abort_warned latch)
}
```

Back-solve (least-squares inverse of X-fin mixer):
```
de = 0.25 * (-f0 - f1 + f2 + f3)
dr = 0.25 * (-f0 + f1 + f2 - f3)
da = 0.25 * ( f0 + f1 + f2 + f3)
```

هذا يعكس الـ forward mixer الموجود في RocketMPC.cpp:1526-1529:
```
fin0 = da - de - dr
fin1 = da - de + dr
fin2 = da + de + dr
fin3 = da + de - dr
```

**ج) تعبئة التليمتري** قبل `publish(status)`:
```cpp
status.can_stale_us      = _can_stale_us;
status.can_abort_events  = _can_abort_events;
status.first_fb_received = _first_fb_received ? 1 : 0;
status.valid_mask        = _can_valid_mask;
```

### 4.6 Airframes

- `22003_m130_rocket_mpc` (SITL): **لا تعديل** → `ROCKET_USE_SRV_FB=0` (من default).
- `22004_m130_rocket_mpc_hitl`: أُضيف `param set ROCKET_USE_SRV_FB 1`.
- `22005_m130_rocket_mpc_real`: أُضيف `param set ROCKET_USE_SRV_FB 1`.

## 5. ما لم يُمَس (مقصوداً)

- `MpcController::_forward_guess` (warm-start في `mpc_controller.cpp:310-317`)
  يستخدم نفس معادلة tau_servo لكنه predictor مستقبلي خاص بالـ solver ولا يؤثر
  على MHE.
- acados OCP + الكود المُولَّد + `m130_ocp_setup.py` + `m130_acados_model.py`.
- الثابت `SOLVER_TAU_SERVO_S = 0.015f` و `SOLVER_DELTA_MAX_RAD` في
  `RocketMPC.cpp`.
- forward X-fin mixer و per-fin clamping في `RocketMPC.cpp` (السطور 1513+).
- 6DOF_v4_pure / hil_config.yaml.

## 6. لا حاجة لإعادة توليد الـ solver

التغيير كله في **طبقة التشغيل** (runtime):
- إضافة حقول `.msg` → PX4 build يُعيد توليد uORB headers تلقائياً.
- إضافة باراميتر → يُلتقط تلقائياً من `params_c`.
- تعديلات C++ → compile عادي.

**لا** `acados_template` regen. **لا** CasADi regen. **لا** تغيير في
`.so`/`.dylib` الخاصة بالـ solver.

## 7. المخاطر والتخفيف

| الخطر | التخفيف |
|---|---|
| CAN glitch < 120ms | Hold لدورة أو اثنتين ثم يستأنف تلقائياً |
| قناة سيرفو واحدة offline | استبدال بآخر قيمة معروفة + back-solve مُتابع (fresh_count=3) |
| ≥2 قنوات offline | Hold كامل + `valid_mask=0x00` ظاهر في telemetry |
| CAN ميت > 500ms | `mavlink_log_critical` + `can_abort_events++` + stale_us ظاهر |
| كسر SITL (22003) | محمي: الباراميتر default=0 → الكتلة الحالية تعمل كما هي |
| CAN spike/noise | Rate-limiter 400°/s يرفضه لدورة واحدة |
| كسر acados / warm-start | مستبعد: لم يُلمَس أي من الملفات المُولَّدة أو OCP |

## 8. كيفية الفحص بعد البناء

1. **SITL (22003):** يجب أن يعمل بدون تغيير سلوكي؛ التليمتري يجب أن يعطي
   `can_stale_us=0, can_abort_events=0, first_fb_received=0, valid_mask=0`.
2. **HITL (22004):** بعد Arm + اتصال CAN:
   - `first_fb_received` → 1 خلال ≤100ms.
   - `valid_mask` → 0x0F طوال الطيران الطبيعي.
   - `can_stale_us` → يتقلب بين 0 و ~20ms (فترة CAN loop).
   - `can_abort_events` → 0 طوال الطيران.
3. **Real (22005):** نفس السلوك.
4. **فصل سيرفو متعمد:** `valid_mask` ينتقل إلى 0x0E/0x0D/0x0B/0x07؛ إذا فُصل
   اثنان فأكثر → 0x00 مع بقاء `_de_act/_dr_act/_da_act` على آخر قيمة.
5. **فصل CAN كاملاً > 500ms:** log `mavlink_log_critical` + `can_abort_events++`
   + `can_stale_us` يتصاعد.

## 9. البناء

**لم يُنفَّذ في هذه الجلسة** — المستخدم طلب صراحة: «انت فقط ارفع، انا من سابني».
تم رفع الفرع كما هو للبناء والفحص من جهته.

## 10. المراجع

- خطة التنفيذ الأصلية + جدول ملاحظات المهندس (8/8 مقبولة).
- الفرع: `devin/1776986012-feedback-only-srvfb`.
- Session: https://app.devin.ai/sessions/9fd24ce592704bebbe22b096b59bd98a
