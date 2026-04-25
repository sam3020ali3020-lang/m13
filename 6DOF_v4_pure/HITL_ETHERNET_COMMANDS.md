# دليل أوامر ربط الهاتف عبر Ethernet لـ HITL

هذا الملف يلخص الأوامر التي تم استخدامها فعليا بالترتيب، مع شرح الهدف من كل خطوة والنتيجة المتوقعة.

## 1) فحص الواجهة والادوات

### 1.1 التحقق من واجهات الشبكة
```bash
ip -br link
```
الهدف:
- التأكد من اسم واجهة الايثرنت الصحيحة (كانت `enp129s0`).

### 1.2 التحقق من حالة ADB
```bash
adb devices -l
```
الهدف:
- معرفة هل الهاتف ظاهر حاليا عبر USB او الشبكة.

### 1.3 التحقق من توفر ADB
```bash
adb version
```
الهدف:
- التأكد ان اداة `adb` مثبتة وتعمل.

### 1.4 التحقق من توفر dnsmasq
```bash
dnsmasq --version | head -n 1
```
الهدف:
- التأكد من توفر DHCP server عند الحاجة.

## 2) محاولة الاعداد التقليدي بـ sudo (لم تنجح بسبب كلمة المرور)

### 2.1 فحص IP الحالي للواجهة
```bash
ip -4 -br addr show enp129s0
```
الهدف:
- معرفة هل الواجهة تملك عنوان IP قبل الاعداد.

### 2.2 التحقق من dnsmasq شغال او لا
```bash
ps -ef | rg 'dnsmasq.*enp129s0|dnsmasq --interface=enp129s0' || true
```
ملاحظة:
- `rg` لم يكن مثبتا في هذه البيئة.

### 2.3 اختبار صلاحية sudo بدون Prompt
```bash
sudo -n true && echo SUDO_OK || echo SUDO_NEEDS_PASSWORD
```
النتيجة:
- ظهر `SUDO_NEEDS_PASSWORD`.

### 2.4 محاولة ضبط IP باستخدام sudo
```bash
sudo ip addr flush dev enp129s0 && sudo ip link set enp129s0 up && sudo ip addr add 192.168.2.1/24 dev enp129s0 && ip -4 -br addr show enp129s0
```
النتيجة:
- فشلت بعد 3 محاولات كلمة مرور غير صحيحة.

## 3) الحل البديل الناجح بدون sudo عبر NetworkManager

### 3.1 عرض الاتصالات النشطة
```bash
nmcli -t -f NAME,DEVICE,TYPE connection show --active
```
الهدف:
- معرفة اسم Connection المرتبط بـ `enp129s0` (كان `Wired connection 1`).

### 3.2 فحص حالة ادارة الواجهة
```bash
nmcli device status
```
الهدف:
- التأكد ان الواجهة Managed من NetworkManager.

### 3.3 تثبيت IP يدوي للكمبيوتر
```bash
nmcli connection modify 'Wired connection 1' ipv4.addresses 192.168.2.1/24 ipv4.method manual ipv6.method ignore && nmcli connection up 'Wired connection 1'
```
الهدف:
- اعطاء الكمبيوتر IP ثابت على منفذ الايثرنت.

### 3.4 تفعيل وضع Shared (DHCP تلقائي للهاتف)
```bash
nmcli connection modify 'Wired connection 1' ipv4.method shared ipv4.addresses 192.168.2.1/24 ipv6.method ignore && nmcli connection up 'Wired connection 1'
```
الهدف:
- جعل الكمبيوتر يوزع IP تلقائيا للهاتف بدون تشغيل dnsmasq يدويا.

### 3.5 التحقق من عنوان الواجهة والجيران
```bash
nmcli -t -f NAME,DEVICE,TYPE,STATE connection show --active
ip -4 -br addr show enp129s0
ip neigh show dev enp129s0 || true
```
النتيجة:
- الواجهة اخذت `192.168.2.1/24`.
- ظهر عنوان هاتف مرشح `192.168.2.100`.

## 4) ربط ADB عبر Ethernet

### 4.1 اختبار وصول الشبكة للهاتف
```bash
ping -c 3 -W 1 192.168.2.100 || true
```
النتيجة:
- Ping ناجح.

### 4.2 محاولة اتصال ADB
```bash
adb connect 192.168.2.100:5555
```
النتيجة الاولية:
- `failed to authenticate` وظهر `unauthorized`.

### 4.3 اعادة المحاولة بعد السماح على الهاتف
```bash
adb disconnect 192.168.2.100:5555; adb connect 192.168.2.100:5555; adb devices -l
```
ثم تم استخدام محاولة تلقائية سريعة:
```bash
for i in $(seq 1 20); do adb connect 192.168.2.100:5555 >/dev/null 2>&1; state=$(adb devices | awk '/192.168.2.100:5555/{print $2}'); echo "try $i: ${state:-not_listed}"; [[ "$state" == "device" ]] && break; sleep 1; done; adb devices -l
```
النتيجة النهائية:
- الحالة صارت `device` (اتصال ADB مفعل بالكامل).

## 5) Port Forwarding لـ HITL

```bash
adb forward tcp:5760 tcp:5760 && adb forward --list
```
الهدف:
- تمرير منفذ MAVLink/HITL من الهاتف الى الكمبيوتر.

النتيجة:
- ظهر السطر:
  - `192.168.2.100:5555 tcp:5760 tcp:5760`

## 6) التحقق النهائي

```bash
ping -c 2 192.168.2.100 && adb devices -l && ip -4 -br addr show enp129s0
```
النتيجة:
- Ping ناجح.
- الهاتف ظاهر بحالة `device`.
- الواجهة على `192.168.2.1/24`.

---

## تشغيل سريع في كل مرة (النسخة المختصرة)

بعد تنفيذ `adb tcpip 5555` عبر USB مرة واحدة، استخدم:

```bash
nmcli connection modify 'Wired connection 1' ipv4.method shared ipv4.addresses 192.168.2.1/24 ipv6.method ignore
nmcli connection up 'Wired connection 1'

adb connect 192.168.2.100:5555
adb forward tcp:5760 tcp:5760
adb devices -l
adb forward --list
```

اذا ظهر `unauthorized`:
- وافق على رسالة RSA في شاشة الهاتف (Allow/Always allow)، ثم اعد `adb connect`.
