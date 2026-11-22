# Button and key mapping

The FPGA Companion uses a fixed pattern to send gamepad and joystick direction and
button and keyboard information into the core. The cores are designed to handle these
in a way that they target machine expected and in most cases these defaults work
just fine. However, there are a few exceptions:

- Some small keyboards may lack vital keys
- Some cores don't make use of all gamepad buttons
- Some cores use keys in a rather unconventional way

These cases may be handled using the button and key remapping ability of the
FPGA Companion.

## The OSD hotkey

The most obvious key to remap is the hotkey used to open the on-screen-display. It's
by default mapped to the ```F12``` key which is what the original Minimig used as well as
MiST and MiSTer. Especially small wireless keyboards may not have an ```F12``` key at all.

To use a different key like ```F8``` create a file named ```config.ini``` on your sd
card. This is the same file used for [network settings as well](NETWORKING.md).
Add an entry like this:

```
[menu]
hotkey=F8
```

Insert the SD card into you MiSTle setup and boot. The FPGA Companion will
automatically pick this file up and will use ```F8``` instead of ```F12``` as
the hotkey. Another very logical choice would be the ```Menu``` key which
is present on many keyboards:

```
[menu]
hotkey=Menu
```

## Gamepad buttons

Not all cores make use of all buttons on a joystick or gamepad. E.g. the Amiga will
only use two buttons ```A``` and ```B``` and the remaining buttons like ```Start```
are unused. For WHDLOAD games it may be helpful to map one of the unused buttons to
the ```F10``` key.

> [!IMPORTANT]
> The mapping of gamepad buttons is a very core specific thing and thus takes place
> in the core specific ini file like e.g. ```amiga.ini``` for the NanoMig.

To map the ```Select``` button to the ```F10``` for the NanoMig add the following line
to the ```amiga.ini```:

```
button Select=F10
```

You can use button combinations as the source and add modifier keys to the target key.
To e.g. make ```Start``` and ```Left Shoulder``` pressed simultaneously to act like
Shift-F9 use the following line:

```
button Select:Start=Left Shift:F9
```

## Keyboard remapping

Some keyboard lack certain important keys. This may be addressed using the
key remap feature.

> [!IMPORTANT]
> The re-mapping of key is a very core specific thing and thus takes place
> in the core specific ini file like e.g. ```amiga.ini``` for the NanoMig.

Assuming your keyboard lacks the right meta key (usually the right Windows key)
and you want it to be mapped to Shift-Left Meta use the following mapping:

```
key Left Shift:Left Meta=Right Meta
```

The left shift key will be "magically" released while the right meta key is being
sent so that only the right meta key is reported without shift.
You can still use the other modifier keys with the remapped key. If you e.g.
press Left Shift, Right Shift and Left Meta with the aforementioned mapping the
Amiga will receive the Right Meta key with Right Shift pressed.

As with the gamepad buttons, additional modifier keys can be added to the target
key. E.g. the ZX Spectrum has not dedicated backspace key and instead has the
backspace function on Shift-0. To map this to the actual backspace key on the
PC keyboard use the following line:

```
key Back Space=Left Shift:0
```

The key names are not case-sensitive and spaces and also be omitted. So the
previous line can also be written as:

```
key backspace=leftshift:0
```

The companion will recreate these entries in the ini file whenever the
settings are saved from the OSD so that they aren't lost while saving.

## Supported keys

The following key names can be used in the mappings. Please be aware
that these refer to US American keyboards. The actual labels on
international keyboard may differ and e.g. to refer to the key that is
labeled ```z``` on a German keyboard you'd have to use ```y``` instead
since the z is on a German keyboard where the y is on a US one. Pages
like [these](https://eurkey.steffen.bruentjen.eu/layout.html?lang=en)
may help.

### Character keys

```A```  ```B```  ```C```  ```D```  ```E```  ```F```  ```G```  ```H```
```I```  ```J```  ```K```  ```L```  ```M```  ```N```  ```O```  ```P```
```Q```  ```R```  ```S```  ```T```  ```U```  ```V```  ```W```  ```X```
```Y```  ```Z```
  
### Number keys
```1```  ```2```  ```3```  ```4```  ```5```  ```6```  ```7```  ```8```
```9```  ```0```
  
### Other Keys
```Enter```  ```Esc```  ```Back Space```  ```Tab```  ```Space```
```-```  ```=```  ```[```  ```]```  ```\\```
```;```  ```' ```  ``` ` ```  ```,```  ```.```  ```/```  ```Caps Lock```
```Print Scr```  ```Scroll Lock```  ```Pause```  ```Insert```  ```Home```
```Page Up```  ```Delete```  ```End```  ```Page Down```  ```Menu```  ```Power```

### Special keys only on international (non US) keyboards

```Eur 1``` backslash on some eur keyboards
```Eur 2``` hash on some eur keyboards

### Function keys
```F1```  ```F2```  ```F3```  ```F4```  ```F5```  ```F6```  ```F7```  ```F8```
```F9```  ```F10```  ```F11```  ```F12```

### Cursor keys
```Right```  ```Left```  ```Down```  ```Up```

### Number pad
```Num Lock```  ```KP /```  ```KP *```  ```KP -```  ```KP +```  ```KP Enter```
```KP 1```  ```KP 2```  ```KP 3```  ```KP 4```  ```KP 5```  ```KP 6```  ```KP 7```
```KP 8```  ```KP 9```  ```KP 0```  ```KP .```  ```KP =```

### Modifier keys
```Left Ctrl```  ```Left Shift```  ```Left Alt```  ```Left Meta```
```Right Ctrl```  ```Right Shift```  ```Right Alt```  ```Right Meta```

## Supported gamepad/joystick buttons

```Right``` ```Left``` ```Down``` ```Up```
```A``` ```B``` ```X``` ```Y```
```Left Shoulder``` ```Right Shoulder```
```Select``` ```Start```
