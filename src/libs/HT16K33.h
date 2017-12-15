#include "mbed.h"

class HT16K33 {
    public: 
        HT16K33(mbed::I2C *i2c, char addr);

        bool init();
        bool enable();
        bool set_brightness(char);
        bool set_blinking(char);
        bool flush_int();
        void setmem(int addr, char val);
        char getmem(int addr);
        bool repaint();

    private:
        mbed::I2C *_i2c;
        char _addr;
        char obuf[17];
        char ibuf[6];
};

class SevenSeg: public HT16K33 {
    public: 
        SevenSeg(mbed::I2C *i2c, char addr) : HT16K33(i2c, addr) { };
        void print_nuli();
        void set_colon(bool colon);
        void set_char(int idx, char val);
        void set_dot(int);
        int idx(int);
        void print(int val);
        void print(double val);
        void print(float val);
        void print(float val, const char *fmt);
        void print(const char *buf);
};

class BarGraph: public HT16K33 {
    public: 
        BarGraph(mbed::I2C *i2c, char addr) : HT16K33(i2c, addr) { };
        void set_vals(char[3], char[3]);
        void set_val(int);
        void all_green();
};
