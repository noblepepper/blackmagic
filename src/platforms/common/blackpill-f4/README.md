# Common code for  WeAct Studio F401CC/F401CE/F411CE boards

This directory contains the common code for the following platforms:
- [blackpill-f401cc](./../../blackpill-f401cc/README.md)
- [blackpill-f401ce](./../../blackpill-f401ce/README.md)
- [blackpill-f411ce](./../../blackpill-f411ce/README.md)

| UART TX         | PA2    | PA2                  | USB USART |
| UART RX         | PA3    | PA3                  | USB USART |

## How to Build

In the following commands replace `blackpill-f4x1cx` with the platform you are using, e.g. `blackpill-f401cc`.

To build the code using the default pinout, run:

```sh
cd blackmagic
make clean
make PROBE_HOST=blackpill-f4x1cx
```

or, to use alternative pinout 1, run:

```sh
cd blackmagic
make clean
make PROBE_HOST=blackpill-f4x1cx ALTERNATIVE_PINOUT=1
```

