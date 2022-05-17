# Purpose-Player-2

Purpose player 2  4.0.1-2

this is a open source software,based on FFmpeg,play audio and video
on Windows and Ubuntu/Linux Mint,and it can display "sound river".

there is a directory "ffmpeg-5.0.1-src-4.0.1-2" ,decompress "ffmpeg-5.0.1.tar.xz",
it create a directory "ffmpeg-5.0.1" ,
use files in "ffmpeg-5.0.1-src-4.0.1-2/fftools" to replace same name files in directory "ffmpeg-5.0.1/fftools",
use files in "ffmpeg-5.0.1-src-4.0.1-2/libavdevice" to replace same name files in directory "ffmpeg-5.0.1/libavdevice",
use file "ffmpeg-5.0.1-src-4.0.1-2/configure" to replace same name file in directory "ffmpeg-5.0.1",
and compile it like FFmpeg, copy ffplay(.exe) to "app" directory,then you can play media file.

it can display "sound river",it separate audio to 117 parts by frequency,display it like a river,
(use mouse click button "River On"),in the bottom is 20khz audio,in the top is 20hz audio,river moves from left to right.

it can process 2.0 channel audio and 5.1 channel audio,only display left channel(1 channel) audio.

you can goto http://ffmpeg.org/download.html#release_5.0 to download ffmpeg-5.0.1.tar.xz, or you can download in this web page.

How to compile:

on windows, you need install Msys+Mingw/Msys2+MinGW64, 
on Ubuntu or Linux Mint you need install gcc,

decompress yasm-1.3.0.tar.gz and run

    ./configure
    make -j
    sudo make install

decompress SDL-1.2.15.src.tar.gz and run

    ./configure
    make -j
    sudo make install

then goto directory "ffmpeg-5.0.1" and run 

    ./configure
    make -j

copy ffplay(.exe) to app directory

you can run ffplay in Ubuntu 12.04 ,

if your Ubuntu is 14.04/16.04 (32bit),you need install 32bit SDL,decompress SDL-1.2.15-1.i386.rpm ,
copy all file in ./usr/lib to /usr/lib . (sudo cp -r ./usr/* /usr)

if you use Ubuntu 18.04/20.04/22.04 ,you need install 64bit SDL,decompress SDL-1.2.15-1.x86_64.rpm,
rename "./usr/lib64" to "./usr/lib" , copy all file in ./usr/lib to /usr/lib . (sudo cp -r ./usr/* /usr)

you can goto http://www.libsdl.org/download-2.0.php to download these files,

ffmpeg-5.0.1 use SDL2, but I modified source code ,ffplay still use SDL1 .

In Ubuntu,if you just compiled ffplay, and want to run it , you need goto directory "SDL-1.2.15",
run command "sudo make uninstall"(unload development library).

On windows or Ubuntu 12.04/14.04/16.04(32bit)/22.04,you can double click "ffplay(.exe)" icon to run it ,if you are on 
Ubuntu 18.04/20.04 , you need open terminal window, move to "app" directory ,input command "./ffplay" to run it .


小戴媒体播放器2  4.0.1-2
 
是一个开源软件,全媒体,包括视频,音频,跨平台(WindowsXP/Vista/7/10,Ubuntu/Linux Mint),可以显示声音河流 .
 
这里有目录ffmpeg-5.0.1-src-4.0.1-2,解压ffmpeg-5.0.1.tar.xz,将创建ffmpeg-5.0.1目录, 
用ffmpeg-5.0.1-src-4.0.1-2/fftools里的文件,替换掉ffmpeg-5.0.1/fftools里的同名文件,
用ffmpeg-5.0.1-src-4.0.1-2/libavdevice里的文件,替换掉ffmpeg-5.0.1/libavdevice里的同名文件,
用ffmpeg-5.0.1-src-4.0.1-2/configure文件,替换掉ffmpeg-5.0.1里的同名文件,
按照原来一样的方法编译,然后把ffplay(.exe)拷贝到app目录.
 
可以显示声音河流,把声音按照频率分成117段,象显示一条河流一样显示出来,
(点击窗口右下角的"River On"),最下面的是20KHz,最上面的是20Hz,河流从左向右流动,

能显示2.0声道的媒体文件,能显示5.1声道电影文件(只提取左声道一个声道的声音),

你可以在 http://ffmpeg.org/download.html#release_5.0 下载 ffmpeg-5.0.1.tar.xz ，或者在当前页面下载。
 
编译方法如下：

如果是Windows平台，需要先安装Msys+Mingw/Msys2+MinGW64,如果是Ubuntu或者Linux Mint需要先安装gcc
 
把 yasm-1.3.0.tar 展开
在 yasm-1.3.0 目录下运行：

    ./configure
    make -j
    sudo make install
 
把 SDL-1.2.15.src.tar 展开
在 SDL-1.2.15 目录下运行：

    ./configure
    make -j
    sudo make install
 
在 ffmpeg-5.0.1 目录下运行：

    ./configure
    make -j

把ffplay(.exe)拷贝到app目录.
 
在 Ubuntu 12.04 可以直接运行,或者用命令行运行(有更多提示，显示媒体信息).

Ubuntu 14.04/16.04(32bit) 下面需要安装32bit SDL运行时库,解压SDL-1.2.15-1.i386.rpm ,把 usr/lib 下面
所有文件拷贝到/usr/lib(在命令行运行 sudo cp -r ./usr/* /usr) .

Ubuntu 18.04/20.04/22.04下面需要安装64bit SDL运行时库,解压SDL-1.2.15-1.x86_64.rpm ,
把 usr/lib64 改为usr/lib, 把 usr/lib 下面所有文件拷贝到/usr/lib (在命令行运行 sudo cp -r ./usr/* /usr) .

你可以在 http://www.libsdl.org/download-2.0.php 下载这些文件 ，

ffmpeg-5.0.1 使用 SDL2, 但是我修改了源代码 ,ffplay 仍然使用 SDL1 .

如果你在 Ubuntu 刚刚编译了ffplay,想要运行ffplay ,需要在 SDL-1.2.15 目录下运行sudo make uninstall.(把开发库卸载)

在 Windows 或者 Ubuntu 12.04/14.04/16.04(32bit)/22.04上面 , 双击ffplay(.exe)图标就可以运行 ,在 Ubuntu 18.04/20.04上面,
你需要打开Terminal窗口 ,移动到app目录 ,输入命令"./ffplay" 去运行 .


