# Actuator Encoding Unification — HITL / Real Flight

**Date:** 2026-04-23  
**Files changed:** `RocketMPC.cpp`, `XqpowerCan.cpp`, `servo_usb_output.cpp`  
**Author:** Code review + fix session

---

## 1. المشكلة

### الوضع قبل التعديل

كان `RocketMPC.cpp` يختار مسار النشر بناءً على `sim_path`:

```
sim_path = _hitl || (SITL_GPS == 1)
```

| | HITL / SITL | Real Flight |
|--|-------------|-------------|
| **Topic** | `actuator_outputs_sim` | `actuator_servos` |
| **الوحدة** | Radians (قيمة خام من الـ solver) | Normalized [-1, +1] |
| **التحويل في XqpowerCan** | `val × (180/π)` | `val × _angle_limit` |

**لماذا كان هذا خطراً؟**

الكود يعمل رياضياً بشكل صحيح (كلا المسارين يصلان للزاوية نفسها)، لكن:

1. **خطر التعديل المستقبلي:** أي مطور يغير `_angle_limit` أو `SOLVER_DELTA_MAX_RAD` في
   مكان واحد دون الآخر يكسر أحد المسارين بصمت — بدون assertion أو compile error.

2. **اختبار HITL لا يغطي Real path:** لأن المسار مختلف، اجتياز HITL لا يضمن صحة
   السلوك في الرحلة الحقيقية.

3. **Cognitive load:** وحدتان مختلفتان في توبيكين مختلفين لنفس الكمية الفيزيائية —
   يجعل قراءة الكود وتتبع الأخطاء أصعب.

4. **`actuator_outputs_sim` بالراديان في real flight:** لو وصل هذا التوبيك خطأً
   لـ XqpowerCan في وضع real، ينتج `fin = 0.3491 rad × (180/π) = 20°` — يبدو صحيحاً!
   لكن أي قيمة أصغر (مثلاً 0.1 rad) تنتج `5.7°` بدل `5.73°` — خطأ صغير غير ملحوظ.
   الخطر الحقيقي: لو نُشر خطأ بأرقام normalized (مثلاً 0.8) على هذا التوبيك في
   حالة HITL → `0.8 × 57.3 = 45.8°` ← أكبر بكثير من الحد المسموح (20°).

---

## 2. الجذر التقني لسبب المسارين

`actuator_outputs_sim` موجود **بسبب قيد معماري في PX4 Simulator**:

```
SimulatorMavlink  ←─── lockstep ───→  actuator_outputs_sim
(Python sim)                          (PX4 internal topic)
```

`SimulatorMavlink` يبلّغ PX4 بـ IMU tick، ثم ينتظر نشر `actuator_outputs_sim` قبل
أن يرسل الـ tick التالي. هذا الـ lockstep لا يمكن تغيير اسم التوبيك فيه بدون تعديل
`SimulatorMavlink` نفسه.

**المشكلة:** لأن `SimulatorMavlink` يتوقع هذا التوبيك، كان `RocketMPC` يكتب
الزوايا (بالراديان) عليه مباشرة في وضع HITL — وبالتالي يحتاج `XqpowerCan`
لقراءتها بوحدات مختلفة.

---

## 3. الحل

### المبدأ

- `actuator_outputs_sim` يبقى **للـ lockstep فقط** — لا يُقرأ للتحكم.
- `actuator_servos` يصبح **المصدر الوحيد** للتحكم في جميع الأوضاع.

### التغييرات

---

### 3.1 — `RocketMPC.cpp`

**قبل:**
```cpp
const bool sim_path = _hitl || (_param_sitl_gps.get() == 1);

if (sim_path) {
    actuator_outputs_s ao{};
    ao.output[0] = fin[0];   // RADIANS
    ao.output[1] = fin[1];
    ao.output[2] = fin[2];
    ao.output[3] = fin[3];
    _actuator_outputs_sim_pub.publish(ao);
} else {
    float n = fin[i] * (1.0f / SOLVER_DELTA_MAX_RAD);  // NORMALIZED
    as.control[i] = n;
    _actuator_servos_pub.publish(as);
}
```

**بعد:**
```cpp
// --- actuator_servos (normalized) — في جميع الأوضاع ---
for (int i = 0; i < 4; ++i) {
    float n = fin[i] * inv_max_d;   // NORMALIZED [-1, +1]
    n = constrain(n, -1.0f, +1.0f);
    as.control[i] = n;
}
_actuator_servos_pub.publish(as);   // دائماً

// --- actuator_outputs_sim (radians) — للـ lockstep فقط ---
if (_hitl || (_param_sitl_gps.get() == 1)) {
    actuator_outputs_s ao{};
    ao.output[0] = fin[0];   // RADIANS — للـ SimulatorMavlink فقط
    _actuator_outputs_sim_pub.publish(ao);
}
```

---

### 3.2 — `XqpowerCan.cpp`

**قبل:**
```cpp
bool got_sim = _sim_mode && _actuator_outputs_sim_sub.update(&sim_out);

if (got_sim) {
    float angle_deg = val * (180.0f / M_PI);   // RADIANS → DEGREES
    servo_set_position(i, angle_deg);
} else {
    float angle_deg = val * _angle_limit;       // NORMALIZED → DEGREES
    servo_set_position(i, angle_deg);
}
```

**بعد:**
```cpp
// Drain (discard) actuator_outputs_sim — لا نستخدمه للتحكم
{ actuator_outputs_s _discard; _actuator_outputs_sim_sub.update(&_discard); }

// مسار واحد فقط
if (_actuator_servos_sub.update(&servos)) {
    float angle_deg = val * _angle_limit;       // NORMALIZED → DEGREES دائماً
    servo_set_position(i, angle_deg);
}
```

---

### 3.3 — `servo_usb_output.cpp`

**قبل:**
```cpp
// Priority 1: actuator_servos → fin = val × scaling_limit_deg
// Priority 2: actuator_outputs_sim → fin = val × RAD2DEG
```

**بعد:**
```cpp
// Priority 1: actuator_servos → fin = val × scaling_limit_deg
// (Priority 2 removed — actuator_outputs_sim drained silently)
{ actuator_outputs_s _discard{}; sim_out_sub.update(&_discard); }
```

---

## 4. التحقق الرياضي الكامل

### الثوابت

| اسم | قيمة |
|-----|------|
| `SOLVER_DELTA_MAX_RAD` | `0.3490658503988659f` rad (= 20° بالضبط) |
| `inv_max_d` | `1.0f / 0.3490658503988659 = 2.864788975654116` |
| `_angle_limit` | `20.0f` degrees (= `XQCAN_LIMIT` param, default 20°) |
| CAN scale | `18` units/degree (hardware constant) |

---

### المسار الجديد الموحد (HITL + Real)

```
fin[i]  [rad]
  ↓ × inv_max_d  (= 2.8648)
n  [normalized, -1..+1]
  ↓ × _angle_limit  (= 20.0°)
angle_deg  [degrees]
  ↓ × 18  (CAN register scale)
position  [int16 CAN units]
```

**التحقق بالأرقام:**

| fin (rad) | × 2.8648 = n | × 20° = angle_deg | × 18 = CAN position |
|-----------|--------------|-------------------|---------------------|
| +0.3491   | +1.0000      | +20.00°            | +360               |
| +0.1745   | +0.5000      | +10.00°            | +180               |
| 0.0000    | 0.0000       | 0.00°             | 0                  |
| −0.1745   | −0.5000      | −10.00°            | −180               |
| −0.3491   | −1.0000      | −20.00°            | −360               |

**خطأ الحساب:** صفر — المعاملان المتتاليان (`× 2.8648` ثم `× 20`) يساويان (`× 57.2958`)
وهو بالضبط `180/π`:

$$n \times \frac{1}{0.3491} \times 20 = n \times 57.296 = n \times \frac{180}{\pi}$$

---

### مقارنة المسار القديم والجديد

#### القديم — HITL

```
fin = 0.3491 rad
  ↓ publish to actuator_outputs_sim (RADIANS)
  ↓ × (180/π) = × 57.296   [in XqpowerCan HITL branch]
= 20.00°
  ↓ × 18
= 360 (CAN)
```

#### القديم — Real Flight

```
fin = 0.3491 rad
  ↓ × (1/0.3491) = × 2.8648   [in RocketMPC]
= 1.000 normalized
  ↓ × 20°   [in XqpowerCan]
= 20.00°
  ↓ × 18
= 360 (CAN)
```

#### الجديد — موحد

```
fin = 0.3491 rad
  ↓ × (1/0.3491) = × 2.8648   [in RocketMPC — دائماً]
= 1.000 normalized
  ↓ × 20°   [in XqpowerCan — دائماً]
= 20.00°
  ↓ × 18
= 360 (CAN)
```

**النتيجة:** ثلاثة مسارات تعطي نفس `position = 360` CAN units ✅

---

## 5. لماذا يبقى `actuator_outputs_sim` في الكود؟

`SimulatorMavlink` (جزء من PX4 الداخلي) يعمل بنظام lockstep:

```
[Python sim]                    [PX4]
    │                              │
    │─── HIL_SENSOR (IMU tick) ───→│
    │                              │  ← RocketMPC يحسب
    │                              │  ← ينشر actuator_outputs_sim
    │←── HIL_ACTUATOR_CONTROLS ───│  ← SimulatorMavlink يقرأه ويرسله
    │                              │
    │─── HIL_SENSOR (next tick) ──→│  ← tick التالي يأتي فقط بعد النشر
```

إذا لم يُنشر `actuator_outputs_sim`، يتوقف الـ simulation نهائياً.  
**الحل:** نبقيه مع الراديان للـ lockstep، لكن `XqpowerCan` يقرأه ويتجاهله (drain).

---

## 6. نقاط مراجعة التكامل

| تحقق | النتيجة |
|------|---------|
| `SOLVER_DELTA_MAX_RAD = 20°` → `inv_max_d × _angle_limit = 57.296 = 180/π` | ✅ |
| HITL و Real يعطيان نفس CAN position لأي fin ∈ [−0.3491, +0.3491] | ✅ |
| `actuator_outputs_sim` لا يزال يُنشر → lockstep لا يتوقف | ✅ |
| `XqpowerCan` يُفرّغ `actuator_outputs_sim` → لا تراكم في uORB queue | ✅ |
| `servo_usb_output` يُفرّغ `actuator_outputs_sim` → نفس الضمان | ✅ |
| `_reverse_mask` يُطبَّق بعد التحويل → لا تغيير في موضع التطبيق | ✅ |
| Clamp ±1.0 في RocketMPC + Clamp ±`_angle_limit` في `servo_set_position` → double-safe | ✅ |

---

## 7. مخطط المسار النهائي

```
┌─────────────────────────────────────────────────────────────────────┐
│                         RocketMPC.cpp                               │
│                                                                     │
│  acados solver                                                      │
│  → fin[0..3] ∈ [−0.3491, +0.3491] rad                             │
│       │                                                             │
│       ├── × (1/0.3491) → n ∈ [−1, +1]                             │
│       │     → actuator_servos.control[0..3]  ← HITL & Real both   │
│       │                                                             │
│       └── raw radians (HITL/SITL only)                             │
│             → actuator_outputs_sim.output[0..3]  ← lockstep only  │
└──────────────────────────┬──────────────────────────────────────────┘
                           │ uORB
           ┌───────────────┴────────────────┐
           ▼                                ▼
┌──────────────────────┐      ┌─────────────────────────┐
│   XqpowerCan.cpp     │      │  servo_usb_output.cpp   │
│                      │      │                         │
│  drain sim_topic     │      │  drain sim_topic        │
│  val × 20° = deg     │      │  val × 20° = deg        │
│  → servo_set_pos()   │      │  → encode_servo()       │
│    × 18 → CAN int16  │      │    → USB frame          │
└──────────────────────┘      └─────────────────────────┘
```

---

## 8. ما لم يتغير

- معامل الـ CAN register: `18 units/degree` — لم يتغير
- `_angle_limit = 20°` — لم يتغير
- `_reverse_mask` — لم يتغير، يُطبَّق كما هو داخل `servo_set_position`
- معادلة `servo_set_position` نفسها — لم تتغير
- الـ lockstep — يعمل كما كان
- مسار الـ feedback (SRV_FB من XqpowerCan) — لم يتغير
