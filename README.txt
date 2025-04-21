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

С��ý�岥����2 6.1.1-4

��һ����Դ���,ȫý��,������Ƶ,��Ƶ,��ƽ̨(Windows 7/10,Ubuntu),������ʾ�������� .

������Ŀ¼ffmpeg-7.1.1-src-6.1.1-4,��ѹffmpeg-7.1.1.tar.xz,������ffmpeg-7.1.1Ŀ¼, ��ffmpeg-7.1.1-src-6.1.1-4/fftools����ļ�,�滻��ffmpeg-7.1.1/fftools���ͬ���ļ�, ����ԭ��һ���ķ�������,Ȼ���ffplay(.exe)������appĿ¼.

������ʾ��������,����������Ƶ�ʷֳ�117��,����ʾһ������һ����ʾ����, (����������½ǵ�"River On"),���������20KHz,���������20Hz,����������������,

����ʾ2.0������ý���ļ�,����ʾ5.1������Ӱ�ļ�(ֻ��ȡ������һ������������),

������� http://ffmpeg.org/download.html ���� ffmpeg-7.1.1.tar.xz �������ڵ�ǰҳ�����ء�

���뷽�����£�

�����Windowsƽ̨����Ҫ�Ȱ�װMsys64+MinGW64,

�����Ubuntu 18.04/20.04/22.04/24.04 or Linux Mint 20/21.1 or Debian 12 ��Ҫ�Ȱ�װgcc:

    sudo apt-get update
    sudo apt-get install build-essential
    
Ȼ��װSDL2

    sudo apt-get update
    sudo add-apt-repository universe
    sudo apt-get install libsdl2-2.0
    sudo apt-get install libsdl2-dev

���ϵͳ˵ : E: Package 'libsdl2-dev' has no installation candidate

���ٴ���������:

    sudo apt-get update
    sudo add-apt-repository universe

Ȼ���ٴΰ�װSDL2:

    sudo apt-get install libsdl2-dev

����㻹�ǲ��ܰ�װSDL2,��������Դ����SDL2-2.30.12.tar.gz,������� http://www.libsdl.org/download-2.0.php ������Щ�ļ�,�����ڵ�ǰҳ�����ء�

�� SDL2-2.30.12.tar.gz չ�� �� SDL2-2.30.12 Ŀ¼�����У�

    ./configure
    make -j
    sudo make install

Ȼ��: �� yasm-1.3.0.tar չ�� �� yasm-1.3.0 Ŀ¼�����У�

    ./configure
    make -j
    sudo make install

�� ffmpeg-7.1.1 Ŀ¼�����У�

    ./configure
    make -j

��ffplay(.exe)������appĿ¼. ffplay(.exe) �� ffmpeg-7.1.1 Ŀ¼��

������ffplay(.exe)֮ǰ,�������Դ���밲װSDL2,��Ҫ�� SDL2-2.30.12 Ŀ¼������"sudo make uninstall"(�ѿ�����ж��).

�� Windows ���� Ubuntu 14.04/16.04(32bit)/22.04/24.04/Linux Mint 21/Debian 12���� , ˫��ffplay(.exe)ͼ��Ϳ������� ,�� Ubuntu 18.04/20.04/Linux Mint 20 ���� ,����Ҫ��Terminal���� ,�ƶ���appĿ¼ ,��������"./ffplay" ȥ���� .

4.0.1-17�汾ʹ��SDL2,���Ըı䴰�ڴ�С,֧��������,֧����������.

4.0.1-20�汾������ʾutf8�ַ�.

4.0.1-25�汾���˺ܶ�.

4.1.2-1�汾֧��8K��ʾ��,�и��õ�utf8�ַ�����.

4.1.2-2�汾�и��õ�utf8�ַ�����.

4.1.2-7�汾�и��õ��ļ����Ƚ�.

5.0-4�汾��5.0-7�汾�ĵ���4��bug,���ǹ���audio resample��.

5.0-21�汾��������GUI.

5.1-3�汾�� mini GUI lib.

5.1.1-10�汾��lib automatic return ���ڵ��ļ���ͨ���ܳ�,��Ļ��С,lib automatic return����õķ�����һ�нϳ����ļ����ֳɼ���

5.1.1-18�汾�и��õ�GUI.

6.1-40-n�汾��ģ�黯��mini GUI Lib.

6.1.1-4�汾��ffmpeg-7.1.1


