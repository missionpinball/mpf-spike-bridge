THIS REPO IS DEPRECATED. MPF USES THE NEW MPF-SPIKE (https://github.com/missionpinball/mpf-spike) BRIDGE WRITTEN IN RUST INSTEAD.


# mpf-spike-bridge
Bridge between the Mission Pinball Framework and Stern SPIKE pinball machines.

This is the code that runs on the SD card installed in the SPIKE CPU node which
allows a remote system (such as MPF) to control the pinball machine.

Details & Instructions:
http://docs.missionpinball.org/en/latest/hardware/spike

## Compiling the bridge (if you made changes)

You need gcc for arm and musl libc to compile the bridge:

    git clone git://git.musl-libc.org/musl
    cd musl
    sudo apt-get install gcc-arm-linux-gnueabi
    CC=/usr/bin/arm-linux-gnueabi-gcc ./configure
    make
    sudo make install

Afterwards, you can simply compile the bridge:

    /usr/local/musl/bin/musl-gcc --static bridge.c -Wall -o bridge
