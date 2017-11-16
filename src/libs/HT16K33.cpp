#include "HT16K33.h"

HT16K33::HT16K33(mbed::I2C *i2c, char addr) {
    _i2c = i2c;
    _addr = (0x70 | (addr & 0x07)) << 1;
    for (int i=0; i<17; i++) {
        obuf[i] = 0;
    }
}

bool
HT16K33::init() {
    return enable() ||
        set_brightness(0x0F) ||
        set_blinking(0x01) ||
        flush_int();
}

bool
HT16K33::enable() {
    // system setup
    // 1. turn on system oscillator
    // 2. ROW/INT set
    // returns true on failure
    return _i2c->write(_addr, "\x21", 1) ||
        _i2c->write(_addr, "\xA0", 1);
}

bool
HT16K33::set_brightness(char level) {
    char cmd = 0xE0 | (level & 0x0F);
    return _i2c->write(_addr, &cmd, 1);
}

bool
HT16K33::set_blinking(char level) {
    char cmd = 0x80 | (level & 0x0F);
    return _i2c->write(_addr, &cmd, 1);
}

bool
HT16K33::flush_int() {
    char cmd = 0x40;
    _i2c->write(_addr, &cmd, 1);
    return _i2c->read(_addr, ibuf, 6);
}

void
HT16K33::setmem(int addr, char val) {
    obuf[addr + 1] = val;
}

char
HT16K33::getmem(int addr) {
    return obuf[addr + 1];
}

bool
HT16K33::repaint() {
    obuf[0] = 0x00;
    return _i2c->write(_addr, obuf, 17);
}

void SevenSeg::print_nuli() {
    set_colon(false); // no colon
    setmem(0, 0x37); // N
    setmem(2, 0x3E); // U
    setmem(6, 0x38); // L
    setmem(8, 0x30); // I
    repaint();
}

void SevenSeg::set_colon(bool colon) {
    setmem(4, colon ? 0x02 : 0x00);
}

int SevenSeg::idx(int oidx) {
    switch(oidx) {
        case 0: return 0;
        case 1: return 2;
        case 2: return 6;
        default: return 8;
    }
}

void SevenSeg::set_dot(int oidx) {
    setmem(idx(oidx), getmem(idx(oidx)) | 0x80);
}

void SevenSeg::set_char(int oidx, char val) {
    int idx = this->idx(oidx);
    switch(val) {
        case ' ': setmem(idx, 0x00); break;
        case '-': setmem(idx, 0x40); break;
        case '0': setmem(idx, 0x3F); break;
        case '1': setmem(idx, 0x06); break;
        case '2': setmem(idx, 0x5B); break;
        case '3': setmem(idx, 0x4F); break;
        case '4': setmem(idx, 0x66); break;
        case '5': setmem(idx, 0x6D); break;
        case '6': setmem(idx, 0x7D); break;
        case '7': setmem(idx, 0x07); break;
        case '8': setmem(idx, 0x7F); break;
        case '9': setmem(idx, 0x6F); break;
        case 'a':
        case 'A': setmem(idx, 0x77); break;
        case 'b':
        case 'B': setmem(idx, 0x7C); break;
        case 'c': setmem(idx, 0x58); break;
        case 'C': setmem(idx, 0x39); break;
        case 'd':
        case 'D': setmem(idx, 0x5E); break;
        case 'e':
        case 'E': setmem(idx, 0x79); break;
        case 'f':
        case 'F': setmem(idx, 0x71); break;
        case 'g':
        case 'G': setmem(idx, 0x3D); break;
        case 'h': setmem(idx, 0x74); break;
        case 'H': setmem(idx, 0x76); break;
        case 'i':
        case 'I': setmem(idx, 0x30); break;
        case 'j':
        case 'J': setmem(idx, 0x1E); break;
        case 'L':
        case 'l': setmem(idx, 0x38); break;
        case 'n': setmem(idx, 0x54); break;
        case 'N': setmem(idx, 0x37); break;
        case 'o': setmem(idx, 0x5C); break;
        case 'O': setmem(idx, 0x3F); break;
        case 'p':
        case 'P': setmem(idx, 0x73); break;
        case 'q':
        case 'Q': setmem(idx, 0x67); break;
        case 'r':
        case 'R': setmem(idx, 0x31); break;
        case 's':
        case 'S': setmem(idx, 0x6D); break;
        case 't':
        case 'T': setmem(idx, 0x78); break;
        case 'u': setmem(idx, 0x1C); break;
        case 'U': setmem(idx, 0x3E); break;
        case 'y':
        case 'Y': setmem(idx, 0x6E); break;
        default: setmem(idx, 0x10); break;
    }
}

void SevenSeg::print(int val) {
    char buf[10];
    int xp = 0;
    if (val > -1000 && val < 10000) {
        sprintf(buf, "%4.4d", val);
        print(buf);
    } else {
        while (val > 100 || -val < -10) {
            val /= 10; xp += 1;
        }
        sprintf(buf, "%dE%d", val, xp);
        print(buf);
    }
}


void SevenSeg::print(double val) {
    print((float)val);
}

void SevenSeg::print(float val) {
    char buf[10];
    if (val <= -1000 || val >= 10000) {
        print((int)val);
    } else {
        sprintf(buf, "%f", val);
        print(buf);
    }
}

void SevenSeg::print(const char *buf) {
    int i, j;
    for (i=0, j=0; i<4 && j<10; j++) {
        if (buf[j] == '\0') break;
        else if (buf[j] == '.') {
            if (i > 0) set_dot(i-1);
        } else if (buf[j] == ':') {
            set_colon(true);
        } else if (buf[j] == '+') {
            set_colon(true);
        } else {
            set_char(i++, buf[j]);
        }
    }
    for (; i<4; i++) {
        set_char(i, ' ');
    }
    repaint();
}

void BarGraph::set_vals(char r[3], char g[3]) {
    // red LEDs
    this->setmem(0, (r[1] & 0xF0)        | (r[0] & 0x0F));
    this->setmem(2, ((r[2] & 0x0F) << 4) | ((r[0] & 0xF0) >> 4));
    this->setmem(4, (r[2] & 0xF0)        | (r[1] & 0x0F));

    this->setmem(1, (g[1] & 0xF0)        | (g[0] & 0x0F));
    this->setmem(3, ((g[2] & 0x0F) << 4) | ((g[0] & 0xF0) >> 4));
    this->setmem(5, (g[2] & 0xF0)        | (g[1] & 0x0F));
}

void BarGraph::set_val(int val) {
    char r[] = {0x00, 0x00, 0x00}, g[] = {0x00, 0x00, 0x00};
    for (int i=0; i<val; i++) {
        r[i/8] |= 0x01 << (i % 8);
    }
    set_vals(r, g);
}

void BarGraph::all_green() {
    char r[] = {0x00, 0x00, 0x00}, g[] = {0xFF, 0xFF, 0xFF};
    set_vals(r, g);
}
