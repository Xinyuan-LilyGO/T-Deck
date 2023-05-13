


<h1 align = "center">✨ LilyGO T-Deck ✨</h1>

## **[English](README.MD) | 中文**


## 1️⃣ 示例预览

```

examples 
├─Keyboard_ESP32C3       # ESP32C3 keyboard I2C slave
├─Microphone             # Noise detection  
└─UnitTest               # Factory hardware unit testing         

```


## 2️⃣ 快速开始

### PlatformIO

1. 安装[VisualStudioCode](https://code.visualstudio.com/) 和 [Python](https://www.python.org/)
2. 在`VisualStudioCode`扩展中搜索`PlatformIO`插件并安装.
3. 安装完成后需要将`VisualStudioCode`重新启动
4. 重新开启`VisualStudioCode`后,选择 `VisualStudioCode` 左上角的 `文件` -> `打开文件夹` ->选择 `T-Deck` 目录
5. 点击 `platformio.ini` 文件,在`platformio` 栏目中取消需要使用的示例行,请确保仅仅一行有效
6. 点击左下角的（✔）符号编译
7. 将板子与电脑USB进行连接
8. 点击（→）上传固件
9. 点击 (插头符号) 监视串行输出


### ArduinoIDE


1. 安装 [ArduinoIDE](https://www.arduino.cc/en/software)
2. 将 `T-Deck/lib` 目录内的所有文件夹拷贝到`<C:\Users\UserName\Documents\Arduino\libraries>`,如果没有`libraries`目录,请新建,请注意,不是拷贝`lib`目录,而是拷贝lib目录里面的文件夹
3. 打开ArduinoIDE -> Tools 
   - Board -> ESP32S3 Dev Module
   - USB CDC On Boot -> Enable   # 注意，在不连接USB的时候你需要将Enable改为Disable，这样USB CDC 不会阻止板子的启动
   - CPU Frequency -> 240MHz
   - USB DFU On Boot -> Disable
   - Flash Mode -> QIO 80MHz
   - Flash Size -> 16MB(128Mb)
   - USB Firmware MSC On Boot -> Disable
   - PSRAM -> OPI PSRAM
   - Partition Scheme -> 16M Flash(3MB APP/9.9MB FATFS)
   - USB Mode -> Hardware CDC and JIAG
   - Upload Mode -> UART0/Hardware CDC
   - Upload Speed -> 921600
4. 插入USB到PC,点击上传<如果无法顺利上传,请保持按压BOOT按键,然后单击RST,然后再点击上传,上传完成时需要点击RST退出下载模式>


## 3️⃣ 提示:

0. 如果启用麦克风,那么板子中间的按钮,GPIO0则不可用.
1. 如果遇到不能上传草图,那么你需要将轨迹球中间按压下去，然后插入USB，这是芯片处于下载模式,此时再点击上传草图.
2. ESP32C3的编程烧录接口在RST按键一侧的6Pin排针那里,顺序从RST按键上方开始数，分别为3V3,GND,RST,BOOT,RX,TX


## 4️⃣ 在哪购买:

1. [AliExpress]()
2. [LilyGo Store]()

## 5️⃣ 引脚分布:

引脚定义请看 [utilities.h](./examples//UnitTest//utilities.h)

