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



С��ý�岥����2  4.0.1-21


 
��һ����Դ���,ȫý��,������Ƶ,��Ƶ,��ƽ̨(Windows 7/10,Ubuntu),������ʾ�������� .
 
������Ŀ¼ffmpeg-5.0.1-src-4.0.1-21,��ѹffmpeg-5.0.1.tar.xz,������ffmpeg-5.0.1Ŀ¼, 
��ffmpeg-5.0.1-src-4.0.1-21/fftools����ļ�,�滻��ffmpeg-5.0.1/fftools���ͬ���ļ�,
����ԭ��һ���ķ�������,Ȼ���ffplay(.exe)������appĿ¼.
 
������ʾ��������,����������Ƶ�ʷֳ�117��,����ʾһ������һ����ʾ����,
(����������½ǵ�"River On"),���������20KHz,���������20Hz,����������������,

����ʾ2.0������ý���ļ�,����ʾ5.1������Ӱ�ļ�(ֻ��ȡ������һ������������),

������� http://ffmpeg.org/download.html#release_5.0 ���� ffmpeg-5.0.1.tar.xz �������ڵ�ǰҳ�����ء�


 
���뷽�����£�

�����Windowsƽ̨����Ҫ�Ȱ�װMsys64+MinGW64,
�����Ubuntu 14.04/16.04/18.04/20.04/22.04 ��Ҫ�Ȱ�װgcc:
 
    sudo apt-get update
    sudo apt-get install build-essential

Ȼ��װSDL2

    sudo apt-get update
    sudo add-apt-repository universe
    sudo apt-get install libsdl2-2.0
    sudo apt-get install libsdl2-dev

���ϵͳ˵ :
    E: Package 'libsdl2-dev' has no installation candidate
    
���ٴ���������:

    sudo apt-get update
    sudo add-apt-repository universe

Ȼ���ٴΰ�װSDL2:

    sudo apt-get install libsdl2-dev

�� yasm-1.3.0.tar չ��
�� yasm-1.3.0 Ŀ¼�����У�

    ./configure
    make -j
    sudo make install
 
�� ffmpeg-5.0.1 Ŀ¼�����У�

    ./configure
    make -j

��ffplay(.exe)������appĿ¼.(�� ffmpeg-5.0.1 Ŀ¼��)
 
�� Windows ���� Ubuntu 14.04/16.04(32bit)/22.04���� , ˫��ffplay(.exe)ͼ��Ϳ������� ,�� Ubuntu 18.04/20.04
���� ,����Ҫ��Terminal���� ,�ƶ���appĿ¼ ,��������"./ffplay" ȥ���� .

  �°汾ʹ��SDL2,���Ըı䴰�ڴ�С,֧��������,֧����������.
  �°汾������ʾutf8�ַ�.


