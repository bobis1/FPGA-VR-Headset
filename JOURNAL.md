---
title: "FPGA Vr headser"
author: "Joseph"
description: "A fpga powered vr headset"
created_at: "2026-09-11"
---



# September 10

My goal with this session is to do substantial research about exactly how to implement my ideas. The idea behind this project is to use an FPGA to take an input from a computer(Or have the "input" preprogrammed honestly I want it to be both). I will also have a RP2040 to handle usb inputs and various other background tasks I don't know enough about VR to understand. I am not sure how difficult it is to read/interpret outputs from an IMU so that is another reason for possibly using a RP2040 in addition to a FPGA. I will be structuring the project in a more specific way than I have in the past. This time I have a research folder with a [Resources.md](Research/Resources.md) ![Basic schematic](JournalPictures/9.10InitialSchem.png). Another part of the project that I am not completely sure about is how to go from a 2D image to a 180 at least projection. I did some research into the sterographic photography. The problem with this is I still don't know exactly what I need to do. The reason I am being so broad with my research right now is that I want to get a good base for everything else that I need to do. After doing some more general research I am going to work on the schematic itself. ![Schematic Later](JournalPictures/9.10SchemLater.png). Another thing I need to take into account is, I need to a way to go from a hdmi input. When I was first researching it I originally thought that a TMDS encoder would work. However when I looked into it further I found that FRL is what HDMI 2.1 and beyond uses. I will probably have to account for that in the project. The problem is that, since this is an open source project I probably cannot use the FRL encoders. I will have to use a TMDS encoder as I found later.

- Recording link
https://lapse.hackclub.com/timelapse/FKwI7hJ7bjrV

**Total time spent: 1:34 hours**