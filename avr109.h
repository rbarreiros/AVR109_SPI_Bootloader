#ifndef __AVR109_H__
#define __AVR109_H__

#define AVR_ENTERPROGRAMMING  'P'  // params:               reply: /r
#define AVR_AUTOINCREMENTADDR 'a'  // params:               reply: Y | N
#define AVR_SETADDRESS        'A'  // params: ADR_H ADR_L   reply: /r
#define AVR_WRITEPROGLOW      'c'  // params: DD            reply: /r
#define AVR_WRITEPROGHIGH     'C'  // params: DD            reply: /r
#define AVR_PAGEWRITE         'm'  // params:               reply: /r
#define AVR_READLOCKBITS      'r'  // params:               reply: DD
#define AVR_READPROGRAM       'R'  // params:               reply: 2 x DD
#define AVR_READDATA          'd'  // params:               reply: DD
#define AVR_WRITEDATA         'D'  // params: DD            reply: /r
#define AVR_CHIPERASE         'e'  // params:               reply: /r
#define AVR_WRITELOCKBITS     'I'  // params: DD            reply: /r
#define AVR_READLOWFUSEBITS   'F'  // params:               reply: DD
#define AVR_READHIGHFUSEBITS  'N'  // params:               reply: DD
#define AVR_READEXTFUSEBITS   'Q'  // params:               reply: DD
#define AVR_LEAVEPROGRAMMING  'L'  // params:               reply: /r
#define AVR_SELECTDEVICETYPE  'T'  // params: DD            reply: /r
#define AVR_READSIGNATURE     's'  // params:               reply: 3 x DD
#define AVR_SUPPORTEDDEVICES  't'  // params:               reply: n x DD  finish: 00 /r
#define AVR_SOFTIDENTIFIER    'S'  // params:               reply: s[7]
#define AVR_SOFTVERSION       'V'  // params:               reply: 2 x DD
#define AVR_PROGTYPE          'p'  // params:               reply: DD
#define AVR_SETLED            'x'  // params: DD            reply: /r
#define AVR_CLEARLED          'y'  // params: DD            reply: /r
#define AVR_EXITBOOTLOADER    'E'  // params:               reply: /r
#define AVR_CHECKBLOCKSUPP    'b'  // params:               reply: 'Y' 2 x DD

#define AVR_STARTBLOCKLOAD    'B'  // For FLASH  - params: 2 x DD 'F' n x DD    reply: /r
                                   // For EEPROM - params: 2 x DD 'E' n x DD    reply: /r

#define AVR_STARTBLOCKREAD    'g'  // For FLASH  - params: 2 x DD 'F'           reply: n x DD
                                   // For EEPROM - params: 2 x DD 'E'           reply: n x DD

#endif
