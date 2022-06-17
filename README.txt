# Purpose-Player-2

Purpose player 2  4.0.1-21



this is a open source software,based on FFmpeg,play audio and video
on Windows and Ubuntu,and it can display "sound river".

there is a directory "ffmpeg-5.0.1-src-4.0.1-21" ,decompress "ffmpeg-5.0.1.tar.xz",
it create a directory "ffmpeg-5.0.1" ,
use files in "ffmpeg-5.0.1-src-4.0.1-21/fftools" to replace same name files in directory "ffmpeg-5.0.1/fftools",
and compile it like FFmpeg, copy ffplay(.exe) to "app" directory,then you can play media file.

it can display "sound river",it separate audio to 117 parts by frequency,display it like a river,
(use mouse click button "River On"),in the bottom is 20khz audio,in the top is 20hz audio,river moves from left to right.

it can process 2.0 channel audio and 5.1 channel audio,only display left channel(1 channel) audio.

you can goto http://ffmpeg.org/download.html#release_5.0 to download ffmpeg-5.0.1.tar.xz, or you can download in this web page.



How to compile:

on windows, you need install Msys64+MinGW64, 
on Ubuntu 14.04/16.04/18.04/20.04/22.04 you need install gcc,please input below command:

    sudo apt-get update
    sudo apt-get install build-essential
    
and then install SDL2,please input below command:

    sudo apt-get update
    sudo add-apt-repository universe
    sudo apt-get install libsdl2-2.0
    sudo apt-get install libsdl2-dev

if system say :
    E: Package 'libsdl2-dev' has no installation candidate
    
please run below command again:

    sudo apt-get update
    sudo add-apt-repository universe

and install libsdl2-dev again:

    sudo apt-get install libsdl2-dev

decompress yasm-1.3.0.tar.gz and run

    ./configure
    make -j
    sudo make install

then goto directory "ffmpeg-5.0.1" and run 

    ./configure
    make -j

copy ffplay(.exe) to app directory(in directory "ffmpeg-5.0.1")


On windows or Ubuntu 14.04/16.04(32bit)/22.04,you can double click "ffplay(.exe)" icon to run it ,if you are on 
Ubuntu 18.04/20.04 , you need open terminal window, move to "app" directory ,input command "./ffplay" to run it .

  version 4.0.1-17 and later use SDL2,support window resize,support mouse wheel,support volume changing.
  version 4.0.1-20 and later can display utf8 characters.



小戴媒体播放器2  4.0.1-21


 
是一个开源软件,全媒体,包括视频,音频,跨平台(Windows 7/10,Ubuntu),可以显示声音河流 .
 
这里有目录ffmpeg-5.0.1-src-4.0.1-21,解压ffmpeg-5.0.1.tar.xz,将创建ffmpeg-5.0.1目录, 
用ffmpeg-5.0.1-src-4.0.1-21/fftools里的文件,替换掉ffmpeg-5.0.1/fftools里的同名文件,
按照原来一样的方法编译,然后把ffplay(.exe)拷贝到app目录.
 
可以显示声音河流,把声音按照频率分成117段,象显示一条河流一样显示出来,
(点击窗口右下角的"River On"),最下面的是20KHz,最上面的是20Hz,河流从左向右流动,

能显示2.0声道的媒体文件,能显示5.1声道电影文件(只提取左声道一个声道的声音),

你可以在 http://ffmpeg.org/download.html#release_5.0 下载 ffmpeg-5.0.1.tar.xz ，或者在当前页面下载。


 
编译方法如下：

如果是Windows平台，需要先安装Msys64+MinGW64,
如果是Ubuntu 14.04/16.04/18.04/20.04/22.04 需要先安装gcc:
 
    sudo apt-get update
    sudo apt-get install build-essential

然后安装SDL2

    sudo apt-get update
    sudo add-apt-repository universe
    sudo apt-get install libsdl2-2.0
    sudo apt-get install libsdl2-dev

如果系统说 :
    E: Package 'libsdl2-dev' has no installation candidate
    
请再次运行命令:

    sudo apt-get update
    sudo add-apt-repository universe

然后再次安装SDL2:

    sudo apt-get install libsdl2-dev

把 yasm-1.3.0.tar 展开
在 yasm-1.3.0 目录下运行：

    ./configure
    make -j
    sudo make install
 
在 ffmpeg-5.0.1 目录下运行：

    ./configure
    make -j

把ffplay(.exe)拷贝到app目录.(在 ffmpeg-5.0.1 目录下)
 
在 Windows 或者 Ubuntu 14.04/16.04(32bit)/22.04上面 , 双击ffplay(.exe)图标就可以运行 ,在 Ubuntu 18.04/20.04
上面 ,你需要打开Terminal窗口 ,移动到app目录 ,输入命令"./ffplay" 去运行 .

  新版本使用SDL2,可以改变窗口大小,支持鼠标滚轮,支持音量调节.
  新版本可以显示utf8字符.


