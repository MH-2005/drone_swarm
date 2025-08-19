# 🚁 سیستم پرواز گروهی پهپادها (Swarm Simulation)

سیستم پیشرفته شبیه‌سازی پرواز گروهی پهپادها با قابلیت‌های هوشمند و مطابق با الزامات مسابقه.

## 📋 ویژگی‌های کلیدی

### ✨ مراحل اصلی مسابقه
- **مرحله 1**: تشکیل آرایش‌های مختلف (خط، مثلث، مربع، الماس)
- **مرحله 2**: دوران و جابه‌جایی آرایش‌ها
- **مرحله 3**: سیستم رهبر-پیرو با مسیر از CSV  
- **مرحله 4**: حذف رهبر و انتخاب خودکار رهبر جدید

### 🚀 قابلیت‌های پیشرفته
- **مانع‌گریزی ترکیبی**: ArduPilot + الگوریتم‌های هوشمند
- **نمایش بصری زنده**: نمودارهای 2D/3D با matplotlib
- **لاگ‌گیری جامع**: ثبت دقیق تمام رویدادها و آمار
- **تغییر آرایش پویا**: امکان تغییر شکل در حین پرواز
- **انتخاب رهبر هوشمند**: بر اساس موقعیت و وضعیت پهپاد

## 🛠 پیش‌نیازهای سیستم

### نرم‌افزارهای اصلی
```bash
# ArduPilot SITL
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
./Tools/environment_install/install-prereqs-ubuntu.sh -y
./waf configure --board sitl
./waf copter

# Gazebo Harmonic
sudo apt-get update
sudo apt-get install lsb-release wget gnupg
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic
```

### پکیج‌های Python
```bash
pip install -r requirements.txt
```

## 