# Purpose-Player-2

Purpose player 2  6.1.1-4

this is a open source software,based on FFmpeg,play audio and video
on Windows and Ubuntu,and it can display "sound river".

there is a directory "ffmpeg-7.1.1-src-6.1.1-4" ,decompress "ffmpeg-7.1.1.tar.xz",
it create a directory "ffmpeg-7.1.1" ,
use files in "ffmpeg-7.1.1-src-6.1.1-4/fftools" to replace same name files in directory "ffmpeg-7.1.1/fftools",
and compile it like FFmpeg, copy ffplay(.exe) to "app" directory,then you can play media file.

it can display "sound river",it separate audio to 117 parts by frequency,display it like a river,
(use mouse click button "River On"),in the bottom is 20khz audio,in the top is 20hz audio,river moves from left to right.

it can process 2.0 channel audio and 5.1 channel audio,only display left channel(1 channel) audio.

you can goto http://ffmpeg.org/download.html to download ffmpeg-7.1.1.tar.xz, or you can download in this web page.



How to compile:

on windows, you need install Msys64+MinGW64, 

on Ubuntu 18.04/20.04/22.04/24.04 or Linux Mint 20/21.1 or Debian 12 you need install gcc,please input below command:

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

if you still can't install SDL2, you can install SDL2 by source code,
download SDL2-2.30.12.tar.gz (you can goto http://www.libsdl.org/download-2.0.php to download these files,
or you can download it on this web page)

decompress SDL2-2.30.12.tar.gz and 
in directory SDL2-2.30.12 run

    ./configure
    make -j
    sudo make install

and then:
decompress yasm-1.3.0.tar.gz and 
in directory yasm-1.3.0 run

    ./configure
    make -j
    sudo make install

then goto directory "ffmpeg-7.1.1" and run 

    ./configure
    make -j

copy ffplay(.exe) to app directory ,ffplay(.exe) is in directory "ffmpeg-7.1.1" 

before you run ffplay(.exe),if you installed SDL2 by source code,you need goto directory "SDL2-2.30.12",
run command "sudo make uninstall"(unload development library).


On windows or Ubuntu 14.04/16.04(32bit)/22.04/24.04 or Linux Mint 21 or Debian 12,you can double click "ffplay(.exe)" icon to run it ,if you are on 
Ubuntu 18.04/20.04 or Linux Mint 20, you need open terminal window, move to "app" directory ,input command "./ffplay" to run it .

  version 4.0.1-17 use SDL2,supports window resize,supports mouse wheel,supports volume change.
  
  version 4.0.1-20 can display utf8 characters.
  
  version 4.0.1-25 is much faster.

  version 4.1.2-1  supports 8K minitor and has better utf8 character font .

  version 4.1.2-2  has better utf8 character font .

  version 4.1.2-7  has better file name compare .

  version 5.0-4 to version 5.0-7 fixed 4 bugs, all about audio resample .

  version 5.0-21 has real GUI .

  version 5.1-3 has mini GUI lib.

  version 5.1.1-10 has lib automatic return
  in today, file name usually very long, and screen is small,
  lib automatic return separate one line file name to mutiple lines
  in the best way.

  version 5.1.1-18 has better GUI .

  version 6.1-40-n has modularized mini GUI lib.
  
  version 6.1.1-4 is for ffmpeg-7.1.1

小戴媒体播放器2 6.1.1-4

是一个开源软件,全媒体,包括视频,音频,跨平台(Windows 7/10,Ubuntu),可以显示声音河流 .

这里有目录ffmpeg-7.1.1-src-6.1.1-4,解压ffmpeg-7.1.1.tar.xz,将创建ffmpeg-7.1.1目录, 用ffmpeg-7.1.1-src-6.1.1-4/fftools里的文件,替换掉ffmpeg-7.1.1/fftools里的同名文件, 按照原来一样的方法编译,然后把ffplay(.exe)拷贝到app目录.

可以显示声音河流,把声音按照频率分成117段,象显示一条河流一样显示出来, (点击窗口右下角的"River On"),最下面的是20KHz,最上面的是20Hz,河流从左向右流动,

能显示2.0声道的媒体文件,能显示5.1声道电影文件(只提取左声道一个声道的声音),

你可以在 http://ffmpeg.org/download.html 下载 ffmpeg-7.1.1.tar.xz ，或者在当前页面下载。

编译方法如下：

如果是Windows平台，需要先安装Msys64+MinGW64,

如果是Ubuntu 18.04/20.04/22.04/24.04 or Linux Mint 20/21.1 or Debian 12 需要先安装gcc:

    sudo apt-get update
    sudo apt-get install build-essential
    
然后安装SDL2

    sudo apt-get update
    sudo add-apt-repository universe
    sudo apt-get install libsdl2-2.0
    sudo apt-get install libsdl2-dev

如果系统说 : E: Package 'libsdl2-dev' has no installation candidate

请再次运行命令:

    sudo apt-get update
    sudo add-apt-repository universe

然后再次安装SDL2:

    sudo apt-get install libsdl2-dev

如果你还是不能安装SDL2,可以下载源代码SDL2-2.30.12.tar.gz,你可以在 http://www.libsdl.org/download-2.0.php 下载这些文件,或者在当前页面下载。

把 SDL2-2.30.12.tar.gz 展开 在 SDL2-2.30.12 目录下运行：

    ./configure
    make -j
    sudo make install

然后: 把 yasm-1.3.0.tar 展开 在 yasm-1.3.0 目录下运行：

    ./configure
    make -j
    sudo make install

在 ffmpeg-7.1.1 目录下运行：

    ./configure
    make -j

把ffplay(.exe)拷贝到app目录. ffplay(.exe) 在 ffmpeg-7.1.1 目录下

在运行ffplay(.exe)之前,如果你用源代码安装SDL2,需要在 SDL2-2.30.12 目录下运行"sudo make uninstall"(把开发库卸载).

在 Windows 或者 Ubuntu 14.04/16.04(32bit)/22.04/24.04/Linux Mint 21/Debian 12上面 , 双击ffplay(.exe)图标就可以运行 ,在 Ubuntu 18.04/20.04/Linux Mint 20 上面 ,你需要打开Terminal窗口 ,移动到app目录 ,输入命令"./ffplay" 去运行 .

4.0.1-17版本使用SDL2,可以改变窗口大小,支持鼠标滚轮,支持音量调节.

4.0.1-20版本可以显示utf8字符.

4.0.1-25版本快了很多.

4.1.2-1版本支持8K显示器,有更好的utf8字符字体.

4.1.2-2版本有更好的utf8字符字体.

4.1.2-7版本有更好的文件名比较.

5.0-4版本到5.0-7版本改掉了4个bug,都是关于audio resample的.

5.0-21版本有真正的GUI.

5.1-3版本有 mini GUI lib.

5.1.1-10版本有lib automatic return 现在的文件名通常很长,屏幕很小,lib automatic return用最好的方法把一行较长的文件名分成几行

5.1.1-18版本有更好的GUI.

6.1-40-n版本有模块化的mini GUI Lib.

6.1.1-4版本有ffmpeg-7.1.1


