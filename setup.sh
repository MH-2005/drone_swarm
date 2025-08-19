#!/bin/bash

# اسکریپت راه‌اندازی خودکار سیستم پرواز گروهی
# برای Ubuntu 20.04+ و سیستم‌های Debian-based

set -e  # خروج در صورت خطا

echo "🚁 راه‌اندازی سیستم پرواز گروهی پهپادها"
echo "================================================"

# رنگ‌ها برای نمایش بهتر
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # بدون رنگ

# تابع نمایش پیام‌ها
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# بررسی سیستم عامل
if [[ "$OSTYPE" != "linux-gnu"* ]]; then
    print_error "این اسکریپت فقط برای Linux طراحی شده است"
    exit 1
fi

# بررسی دسترسی sudo
if ! sudo -n true 2>/dev/null; then
    print_warning "برای نصب برخی پکیج‌ها به دسترسی sudo نیاز است"
fi

print_status "بررسی وابستگی‌های سیستم..."

# به‌روزرسانی repository
print_status "به‌روزرسانی لیست پکیج‌ها..."
sudo apt update

# نصب پکیج‌های پایه
print_status "نصب پکیج‌های پایه..."
sudo apt install -y \
    python3 \
    python3-pip \
    python3-venv \
    python3-dev \
    git \
    curl \
    wget \
    build-essential \
    cmake \
    pkg-config \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libgtk-3-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev \
    libopenexr-dev \
    libatlas-base-dev \
    gfortran \
    python3-tk

# بررسی نسخه Python
python_version=$(python3 -c 'import sys; print(".".join(map(str, sys.version_info[:2])))')
min_version="3.8"

if (( $(echo "$python_version $min_version" | awk '{print ($1 >= $2)}') )); then
    print_success "Python $python_version یافت شد ✓"
else
    print_error "Python $min_version یا جدیدتر نیاز است. نسخه فعلی: $python_version"
    exit 1
fi

# ایجاد محیط مجازی
print_status "ایجاد محیط مجازی Python..."
if [ ! -d "venv" ]; then
    python3 -m venv venv
    print_success "محیط مجازی ایجاد شد"
else
    print_warning "محیط مجازی از قبل موجود است"
fi

# فعال‌سازی محیط مجازی
print_status "فعال‌سازی محیط مجازی..."
source venv/bin/activate

# ارتقاء pip
print_status "ارتقاء pip..."
pip install --upgrade pip setuptools wheel

# نصب وابستگی‌های Python
print_status "نصب وابستگی‌های Python..."
if [ -f "requirements.txt" ]; then
    pip install -r requirements.txt
    print_success "وابستگی‌های Python نصب شد"
else
    print_warning "فایل requirements.txt یافت نشد، نصب دستی وابستگی‌ها..."
    pip install mavsdk numpy matplotlib pandas asyncio-mqtt opencv-python
fi

# راه‌اندازی Gazebo
print_status "بررسی و نصب Gazebo..."
if ! command -v gz &> /dev/null; then
    print_status "نصب Gazebo Harmonic..."
    
    # اضافه کردن repository
    sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
    
    sudo apt update
    sudo apt install -y gz-harmonic
    
    print_success "Gazebo نصب شد"
else
    print_success "Gazebo از قبل نصب است"
fi

# دانلود و راه‌اندازی ArduPilot
print_status "بررسی ArduPilot..."
if [ ! -d "ardupilot" ]; then
    print_status "دانلود ArduPilot..."
    git clone --depth 1 https://github.com/ArduPilot/ardupilot.git
    cd ardupilot
    
    print_status "نصب وابستگی‌های ArduPilot..."
    git submodule update --init --recursive
    
    # نصب وابستگی‌ها
    ./Tools/environment_install/install-prereqs-ubuntu.sh -y
    
    print_status "کامپایل ArduPilot..."
    ./waf configure --board sitl
    ./waf copter
    
    cd ..
    print_success "ArduPilot آماده است"
else
    print_success "ArduPilot از قبل موجود است"
fi

# ایجاد پوشه‌های مورد نیاز
print_status "ایجاد ساختار پوشه‌ها..."
mkdir -p logs
mkdir -p data
mkdir -p config

# ایجاد فایل مسیر نمونه
print_status "ایجاد فایل مسیر نمونه..."
if [ ! -f "leader_path.csv" ]; then
    cat > leader_path.csv << EOF
latitude,longitude,altitude,timestamp,formation_type,formation_size,speed,wait_time
-35.363261,149.165230,20.0,0.0,triangle,8.0,5.0,2.0
-35.363300,149.165280,25.0,30.0,triangle,8.0,4.0,3.0
-35.363350,149.165330,20.0,60.0,square,10.0,6.0,2.0
-35.363300,149.165280,25.0,90.0,line,12.0,5.0,3.0
-35.363261,149.165230,20.0,120.0,triangle,8.0,4.0,5.0
EOF
    print_success "فایل مسیر نمونه ایجاد شد"
fi

# ایجاد اسکریپت‌های راه‌اندازی
print_status "ایجاد اسکریپت‌های کمکی..."

# اسکریپت راه‌اندازی ArduPilot
cat > start_ardupilot.sh << EOF
#!/bin/bash
echo "🚁 راه‌اندازی ArduPilot SITL..."
cd ardupilot
./Tools/autotest/sim_vehicle.py -v ArduCopter --count=4 --spacing=10 --speedup=1 --console --map
EOF
chmod +x start_ardupilot.sh

# اسکریپت راه‌اندازی Gazebo
cat > start_gazebo.sh << EOF
#!/bin/bash
echo "🌍 راه‌اندازی Gazebo..."
gz sim -v4 empty.sdf
EOF
chmod +x start_gazebo.sh

# اسکریپت اجرای کامل
cat > run_demo.sh << EOF
#!/bin/bash
echo "🎯 اجرای دمو کامل سیستم پرواز گروهی..."
source venv/bin/activate
python run_competition.py --mission complete --enable-visualization
EOF
chmod +x run_demo.sh

print_success "اسکریپت‌های کمکی ایجاد شد"

# تست سریع
print_status "انجام تست‌های اولیه..."

# تست import های Python
python3 -c "
try:
    import asyncio
    import numpy as np
    import matplotlib.pyplot as plt
    print('✓ وابستگی‌های اصلی Python موجود است')
except ImportError as e:
    print(f'✗ خطا در import: {e}')
    exit(1)
"

# بررسی MAVSDK
python3 -c "
try:
    from mavsdk import System
    print('✓ MAVSDK موجود است')
except ImportError:
    print('✗ MAVSDK نصب نشده - از pip install mavsdk استفاده کنید')
"

print_success "تست‌های اولیه موفق"

# نمایش راهنمای نهایی
echo ""
echo "🎉 راه‌اندازی با موفقیت کامل شد!"
echo "=================================="
echo ""
echo "📋 مراحل بعدی:"
echo "1. راه‌اندازی ArduPilot SITL:"
echo "   ./start_ardupilot.sh"
echo ""
echo "2. در terminal جداگانه، راه‌اندازی Gazebo:"
echo "   ./start_gazebo.sh"
echo ""
echo "3. اجرای دمو:"
echo "   ./run_demo.sh"
echo ""
echo "یا اجرای دستی:"
echo "   source venv/bin/activate"
echo "   python run_competition.py --mission complete"
echo ""
echo "📚 برای راهنمای کامل: cat README.md"
echo "🐛 برای عیب‌یابی: python run_competition.py --log-level DEBUG"
echo ""
print_success "سیستم آماده استفاده است! 🚁"