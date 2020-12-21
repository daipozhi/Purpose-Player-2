# Purpose-Player

Purpose player 3.3

  this is a open source software,based on ffmpeg,play audio and video
on Windows and Ubuntu,and it can display "sound river".

  there is a directory "ffmpeg-4.3-src-3.3ok10" ,decompress "ffmpeg-4.3.tar.xz",
it create a directory "ffmpeg-4.3" ,
  use files in "ffmpeg-4.3-src-3.3ok10/fftools" to replace same name files in directory "ffmpeg-4.3/fftools",
  use files in "ffmpeg-4.3-src-3.3ok10/libavdevice" to replace same name files in directory "ffmpeg-4.3/libavdevice",
  use file "ffmpeg-4.3-src-3.3ok10/configure" to replace same name file in directory "ffmpeg-4.3",
  and compile it like ffmpeg, copy ffplay.exe to "app" directory,then you can play media file.

  it can display "sound river",it separate audio to 256 parts by frequency,display it like a river,
(use mouse click button "River On"),in the bottom is 20khz audio,in the top is 20hz audio,river moves from left to right.

  it can process 2.0 channel audio and 5.1 channel audio,only display left channel(1 channel) audio.

  How to compile:
    in windows, you need install msys+mingw, in Ubuntu, every thing is ready,in Linux Mint you need install gcc,

    decompress yasm-1.3.0.tar.gz and run
    ./configure
    make
    sudo make install

    decompress SDL-1.2.15.src.tar.gz and run
    ./configure
    make
    sudo make install

    then goto directory "ffmpeg-4.3" and run 
    ./configure
    make
    sudo make install
    copy ffplay.exe to app directory

  you can run ffplay.exe in Windows and Ubuntu 12.04 ,
  if your Ubuntu is 14.04/16.04,you need install SDL,decompress SDL-1.2.15-1.i386.rpm ,
  copy all file in ./usr/lib to /usr/lib.(sudo cp -r ./usr/* /usr)
  if you use Mint-20-x86-64 ,you need install 64bit SDL,decompress SDL-1.2.15-1.x86_64.rpm,rename "./usr/lib64" to "./usr/lib" ,
  copy all file in ./usr/lib to /usr/lib .(sudo cp -r ./usr/* /usr)

  ffmpeg-4.3 use SDL2, but I can't install SDL2 on Ubuntu or Mint , so I modified source code ,ffplay still use SDL1 .

  In Ubuntu/Mint,if you just compiled ffplay.exe, and want to run it , you need goto directory "SDL-1.2.15",
run command "sudo make uninstall"(unload develop library).



小戴媒体播放器  3.3

 
是一个开源软件,全媒体,包括视频,音频,跨平台(WindowsXP/Vista/7,Ubuntu/Mint),可以显示声音河流 .
 
这里有目录ffmpeg-4.3-src-3.3ok10,解压ffmpeg-4.3.tar.xz,将创建ffmpeg-4.3目录, 
    用ffmpeg-4.3-src-3.3ok10/fftools里的文件,替换掉ffmpeg-4.3/fftools里的同名文件
    用ffmpeg-4.3-src-3.3ok10/libavdevice里的文件,替换掉ffmpeg-4.3/libavdevice里的同名文件
    用ffmpeg-4.3-src-3.3ok10/configure文件,替换掉ffmpeg-4.3里的同名文件
    按照原来一样的方法编译,然后把ffplay.exe拷贝到app目录.
 
可以显示声音河流,把声音按照频率分成256段,象显示一条河流一样显示出来,
(点击窗口右下角的"River On"),最下面的是20KHz,最上面的是20Hz,河流从左向右流动,

能显示2.0声道的媒体文件,能显示5.1声道电影文件(只提取左声道一个声道的声音),

 
编译方法如下：
(如果是Windows平台，需要先安装msys+mingw,如果是Ubuntu可以直接编译,如果是Mint需要先安装gcc)
 
    把 yasm-1.3.0.tar 展开
    在 yasm-1.3.0 目录下运行：
    ./configure
    make
    sudo make install
 
    把 SDL-1.2.15.src.tar 展开
    在 SDL-1.2.15 目录下运行：
    ./configure
    make
    sudo make install
 
    把 ffmpeg-4.3 展开
    在 ffmpeg-4.3 目录下运行：
    ./configure
    make
    sudo make install
    把ffplay拷贝到App目录.
 
Windows Ubuntu 12.04 可以直接运行,或者用命令行运行(有更多提示，显示媒体信息).

Ubuntu 14.04 16.04下面需要安装SDL运行时库,解压SDL-1.2.15-1.i386.rpm ,把 usr/lib 下面
    所有文件拷贝到/usr/lib(在命令行运行 sudo cp -r ./usr/* /usr)
Mint-20-x86-64下面需要安装64bit SDL运行时库,解压SDL-1.2.15-1.x86_64.rpm ,把 usr/lib64 改为usr/lib, 
    把 usr/lib 下面所有文件拷贝到/usr/lib(在命令行运行 sudo cp -r ./usr/* /usr)

ffmpeg-4.3 使用 SDL2, 但是我不能在Ubuntu/Mint安装 SDL2 , 所以我修改了源代码 ,ffplay 仍然使用 SDL1 .

如果你在 Ubuntu 刚刚编译了ffplay,需要在 SDL-1.2.15 目录下运行sudo make uninstall.(把开发库卸载)

