# Purpose-Player-2

Purpose player 2  3.4.1

  this is a open source software,based on FFmpeg,play audio and video
on Windows and Ubuntu/Linux Mint,and it can display "sound river".

  there is a directory "ffmpeg-4.4.1-src-3.4.1" ,decompress "ffmpeg-4.4.1.tar.xz",
  it create a directory "ffmpeg-4.4.1" ,
  use files in "ffmpeg-4.4.1-src-3.4.1/fftools" to replace same name files in directory "ffmpeg-4.4.1/fftools",
  use files in "ffmpeg-4.4.1-src-3.4.1/libavdevice" to replace same name files in directory "ffmpeg-4.4.1/libavdevice",
  use file "ffmpeg-4.4.1-src-3.4.1/configure" to replace same name file in directory "ffmpeg-4.4.1",
  and compile it like FFmpeg, copy ffplay(.exe) to "app" directory,then you can play media file.
  ( if you are in Linux or Msys , copy ffplay(.exe) to app directory , if you are in Msys2(MinGW64) ,
  please copy ffplay.exe to app2 directory )

  it can display "sound river",it separate audio to 117 parts by frequency,display it like a river,
  (use mouse click button "River On"),in the bottom is 20khz audio,in the top is 20hz audio,river moves from left to right.

  it can process 2.0 channel audio and 5.1 channel audio,only display left channel(1 channel) audio.

  How to compile:

  in windows, you need install Msys+Mingw/Msys2+MinGW64, in Ubuntu or Linux Mint you need install gcc,

    decompress yasm-1.3.0.tar.gz and run
    ./configure
    make
    sudo make install

    decompress SDL-1.2.15.src.tar.gz and run
    ./configure
    make
    sudo make install

    then goto directory "ffmpeg-4.4.1" and run 
    ./configure
    make
    sudo make install
    copy ffplay(.exe) to app(2) directory

  you can run ffplay.exe in Windows and Ubuntu 12.04 ,
  if your Ubuntu is 14.04/16.04 (32bit),you need install 32bit SDL,decompress SDL-1.2.15-1.i386.rpm ,
  copy all file in ./usr/lib to /usr/lib . (sudo cp -r ./usr/* /usr)
  if you use Ubuntu 18.04/20.04 or Linux Mint-20-x86-64 ,you need install 64bit SDL,decompress SDL-1.2.15-1.x86_64.rpm,rename 
  "./usr/lib64" to "./usr/lib" ,
  copy all file in ./usr/lib to /usr/lib . (sudo cp -r ./usr/* /usr)

  ffmpeg-4.4.1 use SDL2, but I can't install SDL2 on Ubuntu or Linux Mint , so I modified source code ,ffplay still use SDL1 .

  In Ubuntu/Linux Mint,if you just compiled ffplay.exe, and want to run it , you need goto directory "SDL-1.2.15",
  run command "sudo make uninstall"(unload develop library).

  On windows or Ubuntu 12.04/14.04/16.04 (32bit),you can double click "ffplay(.exe)" icon to run it ,if you are on 
  Ubuntu 18.04/20.04 or Linux Mint 20 , you need open terminal window, move to "app" directory ,input command 
  "./ffplay" to run it .

С��ý�岥����2  3.4.1

 
��һ����Դ���,ȫý��,������Ƶ,��Ƶ,��ƽ̨(WindowsXP/Vista/7/10,Ubuntu/Linux Mint),������ʾ�������� .
 
������Ŀ¼ffmpeg-4.4.1-src-3.4.1,��ѹffmpeg-4.4.1.tar.xz,������ffmpeg-4.4.1Ŀ¼, 
    ��ffmpeg-4.4.1-src-3.4.1/fftools����ļ�,�滻��ffmpeg-4.4.1/fftools���ͬ���ļ�,
    ��ffmpeg-4.4.1-src-3.4.1/libavdevice����ļ�,�滻��ffmpeg-4.4.1/libavdevice���ͬ���ļ�,
    ��ffmpeg-4.4.1-src-3.4.1/configure�ļ�,�滻��ffmpeg-4.4.1���ͬ���ļ�,
    ����ԭ��һ���ķ�������,Ȼ���ffplay(.exe)������appĿ¼.
    (�����Msys/Linuxƽ̨,��ffplay(.exe)������appĿ¼,�����Msys2ƽ̨,��ffplay.exe������app2Ŀ¼)
 
������ʾ��������,����������Ƶ�ʷֳ�117��,����ʾһ������һ����ʾ����,
    (����������½ǵ�"River On"),���������20KHz,���������20Hz,����������������,

����ʾ2.0������ý���ļ�,����ʾ5.1������Ӱ�ļ�(ֻ��ȡ������һ������������),

 
���뷽�����£�

�����Windowsƽ̨����Ҫ�Ȱ�װMsys+Mingw/Msys2+MinGW64,�����Ubuntu����Linux Mint��Ҫ�Ȱ�װgcc
 
    �� yasm-1.3.0.tar չ��
    �� yasm-1.3.0 Ŀ¼�����У�
    ./configure
    make
    sudo make install
 
    �� SDL-1.2.15.src.tar չ��
    �� SDL-1.2.15 Ŀ¼�����У�
    ./configure
    make
    sudo make install
 
    �� ffmpeg-4.4.1 Ŀ¼�����У�
    ./configure
    make
    sudo make install
    ��ffplay(.exe)������app(2)Ŀ¼.
 
Windows/Ubuntu 12.04 ����ֱ������,����������������(�и�����ʾ����ʾý����Ϣ).

Ubuntu 14.04 16.04 (32bit) ������Ҫ��װ32bit SDL����ʱ��,��ѹSDL-1.2.15-1.i386.rpm ,�� usr/lib ����
    �����ļ�������/usr/lib(������������ sudo cp -r ./usr/* /usr) .
Ubuntu 18.04/20.04 ���� Linux Mint-20-x86-64 ������Ҫ��װ64bit SDL����ʱ��,��ѹSDL-1.2.15-1.x86_64.rpm ,
    �� usr/lib64 ��Ϊusr/lib, �� usr/lib ���������ļ�������/usr/lib
    (������������ sudo cp -r ./usr/* /usr) .

ffmpeg-4.4.1 ʹ�� SDL2, �����Ҳ�����Ubuntu/Linux Mint��װ SDL2 , �������޸���Դ���� ,ffplay ��Ȼʹ�� SDL1 .

������� Ubuntu �ոձ�����ffplay,��Ҫ����ffplay ,��Ҫ�� SDL-1.2.15 Ŀ¼������sudo make uninstall.(�ѿ�����ж��)

�� Windows ���� Ubuntu 12.04/14.04/16.04 (32bit)���� , ˫��ffplay(.exe)ͼ��Ϳ������� ,�� Ubuntu 18.04/20.04 ���� 
    Linux Mint 20���� ,����Ҫ��Terminal���� ,�ƶ���appĿ¼ ,��������"./ffplay" ȥ���� .



