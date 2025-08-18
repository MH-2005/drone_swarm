# 🚁 نقشه‌راه جامع پروژه Swarm Simulation

## 📋 مرور کلی پروژه
**هدف**: ساخت سامانه‌ی شبیه‌سازی پرواز گروهی پهپادها با قابلیت‌های پیشرفته
**تکنولوژی**: MAVSDK (Python) + Gazebo + ArduPilot SITL

---

## 🎯 Phase 1: محیط پایه و زیرساخت
### ✅ نیازمندی‌های اصلی
- [ ] راه‌اندازی Gazebo Garden/Fortress
- [ ] نصب ArduPilot SITL (4 پهپاد)
- [ ] تست اتصال MAVSDK Python
- [ ] طراحی معماری ماژولار

### 🏗 معماری پیشنهادی
```
swarm_simulation/
├── core/
│   ├── drone.py          # کلاس تک پهپاد
│   ├── swarm_manager.py  # مدیر گروه
│   └── utils.py          # ابزارهای مشترک
├── formation/
│   ├── shapes.py         # تعریف شکل‌ها
│   └── transforms.py     # دوران و جابه‌جایی
├── navigation/
│   ├── leader_follower.py
│   └── path_manager.py   # مدیریت مسیر CSV
├── avoidance/
│   ├── obstacle_detection.py
│   └── hybrid_avoidance.py
├── visualization/
│   ├── realtime_plot.py
│   └── logger.py
└── competition/
    ├── run_competition.py
    └── mission_executor.py
```

---

## 🔹 Phase 2: تشکیل شکل‌ها (Formation Core)
### الزامات شیوه‌نامه
- [ ] خط، مثلث، مربع
- [ ] پارامترهای CLI (طول، ارتفاع)
- [ ] Runtime Commands

### 🚀 ارتقائات پیشنهادی
- [ ] **Dynamic Formation Change**: تغییر شکل در حین پرواز
- [ ] **Smooth Transitions**: انتقال نرم بین آرایش‌ها
- [ ] **Formation Validation**: چک کردن امکان‌پذیری آرایش قبل اجرا

### مثال Runtime Command
```bash
python run_competition.py --formation square --size 5.0 --altitude 20 --orientation vertical
```

---

## 🔄 Phase 3: تغییر آرایش (Advanced Transformations)
### الزامات شیوه‌نامه
- [x] دوران حول محورهای x,y,z
- [x] جابه‌جایی خطی
- [x] اجرای مجزا یا همزمان دستورات

### 🚀 ارتقائات پیشنهادی
- [ ] **Coordinated Movement**: حرکت هماهنگ تمام پهپادها
- [ ] **Formation Memory**: ذخیره و بازیابی آرایش‌های قبلی
- [ ] **Advanced Rotations**: دوران پیچیده (Euler angles, quaternions)

### مثال Runtime Command
```bash
python run_competition.py --rotate-z 45 --move-x 3.0 --formation-type triangle
```

---

## 👥 Phase 4: رهبر-پیرو هوشمند (Smart Leader-Follower)
### الزامات شیوه‌نامه
- [x] فاصله‌ی 3-10 متر بین پهپادها
- [x] خواندن مسیر از `leader_path.csv`
- [x] فقط رهبر از مسیر مطلع است

### 🚀 ارتقائات پیشنهادی
- [ ] **Adaptive Formation**: تطبیق آرایش با مسیر (مثلاً در پیچ‌ها)
- [ ] **Formation Healing**: بازسازی آرایش بعد از اختلال
- [ ] **Multi-Level Following**: پیروی چندمرحله‌ای (رهبر → زیررهبر → پیرو)
- [ ] **Path Optimization**: بهینه‌سازی مسیر برای کاهش انرژی

### ساختار CSV پیشنهادی
```csv
timestamp,lat,lon,alt,formation_type,formation_size
0.0,-35.363261,149.165230,20.0,triangle,4.0
30.0,-35.363300,149.165280,25.0,square,5.0
60.0,-35.363350,149.165330,20.0,line,6.0
```

---

## 🔄 Phase 5: مدیریت رهبری پیشرفته (Advanced Leadership)
### الزامات شیوه‌نامه
- [x] حذف رهبر با دستور `disarm_leader`
- [x] انتخاب خودکار رهبر جدید
- [x] بازگشت رهبر قبلی به Home

### 🚀 ارتقائات پیشنهادی
- [ ] **Smart Leader Selection**: انتخاب بر اساس موقعیت، باتری، سیگنال
- [ ] **Graceful Handover**: انتقال نرم رهبری بدون وقفه
- [ ] **Emergency Protocols**: واکنش به شرایط اضطراری
- [ ] **Leader History**: ردیابی تاریخچه‌ی تغییرات رهبری

### الگوریتم انتخاب رهبر هوشمند
```python
def select_new_leader(drones, current_path_point):
    candidates = [d for d in drones if d.is_alive()]
    
    scores = {}
    for drone in candidates:
        score = 0
        score += battery_weight * drone.battery_level
        score += proximity_weight * (1/distance_to_path(drone, current_path_point))
        score += stability_weight * drone.flight_stability
        scores[drone.id] = score
    
    return max(candidates, key=lambda d: scores[d.id])
```

---

## 🛡 Phase 6: مانع‌گریزی ترکیبی (Hybrid Obstacle Avoidance)
### امتیاز تشویقی شیوه‌نامه
- [x] عدم برخورد با موانع تصادفی

### 🚀 رویکرد پیشرفته: Hybrid Approach
#### لایه ۱: ArduPilot BendyRuler (پایه‌ای)
```python
# فعال‌سازی obstacle avoidance در ArduPilot
await drone.param.set_param_float("AVOID_ENABLE", 7.0)  # All avoid types
await drone.param.set_param_float("AVOID_DIST_MAX", 10.0)  # 10m detection
```

#### لایه ۲: Formation-Aware Avoidance (هوشمند)
- **Formation Preservation**: حفظ آرایش حین دوری از مانع
- **Coordinated Avoidance**: هماهنگی گروهی در مانع‌گریزی
- **Fallback Strategies**: بازگشت به آرایش بعد از عبور از مانع

---

## 📊 Phase 7: نظارت و تجسم (Monitoring & Visualization)
### 🚀 قابلیت‌های پیشنهادی
- [ ] **Real-time Dashboard**: نمایش زنده موقعیت‌ها و وضعیت
- [ ] **Formation Health Monitor**: سلامت آرایش و فاصله‌ها
- [ ] **Performance Metrics**: آمار عملکرد (دقت، زمان، انرژی)
- [ ] **3D Path Visualization**: نمایش سه‌بعدی مسیر طی‌شده

### ابزارهای پیاده‌سازی
```python
import matplotlib.pyplot as plt
import plotly.graph_objects as go
from matplotlib.animation import FuncAnimation

# Real-time 2D plot
class SwarmVisualizer:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.drones_scatter = None
        
    def update_positions(self, positions, formation_type):
        # Update live plot
        pass
```

---

## 📝 Phase 8: مستندسازی و تحویل
### الزامات شیوه‌نامه
- [x] کدهای اصلی
- [x] فایل اجرایی `run_competition.py`
- [x] README جامع
- [x] مستندات PDF
- [x] ویدئوهای MP4

### 🚀 مستندسازی پیشرفته
- [ ] **Architecture Diagrams**: نمودارهای معماری
- [ ] **Algorithm Explanations**: توضیح الگوریتم‌های کلیدی
- [ ] **Performance Analysis**: تحلیل عملکرد و بهینه‌سازی‌ها
- [ ] **Future Enhancements**: پیشنهادات توسعه

---

## ⏱ Timeline پیشنهادی (2 هفته)
### هفته اول
- **روز 1-2**: Phase 1-2 (محیط پایه + شکل‌ها)
- **روز 3-4**: Phase 3 (تغییر آرایش)
- **روز 5-7**: Phase 4 (رهبر-پیرو)

### هفته دوم
- **روز 8-9**: Phase 5 (مدیریت رهبری)
- **روز 10-11**: Phase 6-7 (مانع‌گریزی + نمایش)
- **روز 12-14**: Phase 8 (مستندسازی + تست نهایی)

---

## 🏆 نکات کلیدی برای امتیاز بالا
1. **کد تمیز و ماژولار**: معماری قابل توسعه
2. **مدیریت خطا**: Handle کردن تمام حالات غیرمنتظره
3. **Performance**: بهینه‌سازی برای 4+ پهپاد
4. **Innovation**: قابلیت‌های خلاقانه فراتر از الزامات
5. **Documentation**: مستندات کامل و حرفه‌ای
6. **Demo Quality**: ویدئوهای باکیفیت با توضیحات

---

## 🔧 نکات فنی مهم
### محدودیت‌های شیوه‌نامه
- ✅ حداکثر 3 دقیقه برای هر دستور
- ✅ فاصله‌ی 3-10 متر بین پهپادها
- ✅ عدم برخورد (منجر به کسر امتیاز)
- ✅ مسیر حداقل 3 نقطه، حداقل 2 دقیقه

### بهینه‌سازی‌های پیشنهادی
- **Async Programming**: استفاده کامل از asyncio
- **Error Recovery**: بازیابی خودکار از خطاها
- **Resource Management**: مدیریت حافظه و CPU
- **Testing**: تست‌های جامع برای تمام سناریوها