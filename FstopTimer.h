/* -*- C++ -*- */

/*
    Copyright (C) 2011-2015 William Brodie-Tyrrell
    william@brodie-tyrrell.org
  
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of   
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>
*/


#ifndef _FSTOP_TIMER_H_
#define _FSTOP_TIMER_H_

#include <Arduino.h>
#include <RotaryEncoder.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <Keypad.h>
#include <FreqMeasure.h>

// EEPROM layout
#define EE_BACKLIGHT 0x00
#define EE_DRYDOWN 0x01
#define EE_DRYAPPLY 0x02
#define EE_ROTARY 0x03
#define EE_STRIPBASE 0x04
#define EE_STRIPSTEP 0x06
#define EE_STRIPCOV 0x08
#define EE_VERSION 0x09
#define EE_WARMUP 0x0A
#define EE_CONFIGTOP 0x10
#define EE_TOP 0x400



/**
 * Definition of a program of exposures
 */
class Program {
    static const int INVALID=0x0000;
    static const int SLOTBITS=7;
    static const int SLOTBASE=0x80;
    static const int TEXTLEN=14;
    static const long MAXMS=999999L;    // ceiling of 1000s
  
public:

    static const int MAXEXP=8;
    static const int FIRSTSLOT=1;
    static const int LASTSLOT=7;

    /// a single exposure step
    class Exposure {
    public:
  
        /// render the exposure settings to the screen (time and text)
        /// @param lin also display linear time (millis)
        void display(LiquidCrystal &disp, bool lin);
        /// render only the time line (bottom row);
        void displayTime(LiquidCrystal &disp, bool lin);
  
        int stops;        // fixed-point, 1/100ths of a stop
        unsigned long ms; // milliseconds to expose (post-compilation, not saved)
        char text[TEXTLEN+1];    // description (only 14 bytes written to EEPROM)
    };

    /// clear all entries, leaving base exposure of 0 stops
    void clear();
  
    /// step accessor
    Exposure &operator[](int which);
  
    /// determine number of valid exposure objects; first = base
    // unsigned char getCount();
  
    /// convert a program from stops to linear time so that it can be execed
    bool compile(char dd, int warmup);
  
    /// save to EEPROM
    /// @param slot slot-number in 1..7
    void save(int slot);
  
    /// load from EEPROM
    /// @param slot slot-number in 1..7
    bool load(int slot);
  
    /// configure the program as a test strip;
    /// is assumed to compile after this.
    void configureStrip(int base, int step, bool cover);

    /// convert hundredths-of-stops to milliseconds
    static unsigned long hunToMillis(int hunst);

private:

    int slotAddr(int slot);
    void compileStripIndiv(char dd, int warmup);
    void compileStripCover(char dd, int warmup);
    bool compileNormal(char dd, int warmup);
    void clipExposures();

    // compilation settings
    bool isstrip, cover;

    /// first exposure is base, rest as dodges/burns
    Exposure exposures[MAXEXP];
};

/**
 * Object for executing a single Exposure under the control of
 * external (serial port) agency.  No user interaction permitted.
 * Operation is polled instead of closed.
 */
class OpenExecutor {
public:

    OpenExecutor(LiquidCrystal &l, char p_e, int &w);

    /// setup ready to begin exposure on next poll()
    void setStops(int stops);

    /// stop exposure
    void reset();

    void poll();

    bool isComplete() const { return complete; }
    bool isOn() const { return on; }

private:
    Program::Exposure expo;
    unsigned long start, mstotal, lastupdate;
    bool on, complete;

    LiquidCrystal &disp;
    char pin_expose;
    int &warmup;
};

/**
 * Object for executing Programs; assumes user control and will
 * poll keyboard for pause/skip/reset events, returning only when the
 * exposure is complete or cancelled.
 *
 * @todo: make this based on OpenExecutor, which requires that
 * to be button- and pause-aware.
 */
class Executor {
public:

    static const unsigned long UPDATE_PERIOD=100000;

    Executor(LiquidCrystal &d, Keypad &k, ButtonDebounce &b, char p_e, int &w);

    void begin();

    /// must call this to define what will be execed
    /// @param p the current program
    void setProgram(Program *p);

    /// set drydown indication
    /// @param d whether to indicate that drydown is applied
    void setDrydown(bool d);

    Program *getProgram() const { return current; }

    /// define which exposure step to do
    void changePhase(unsigned char ph);
    unsigned char getPhase() const { return execphase; }

    /// move onto next phase
    void nextPhase();

    /// do a controlled, closed (no return until complete) exposure
    void expose();

private:

    /// program we're working on
    Program *current;
  
    LiquidCrystal &disp;
    Keypad &keys;
    ButtonDebounce &button;
    char pin_expose;
    int &warmup;

    bool dd;

    /// program phase about to be executed
    unsigned char execphase;
};

/**
 * Serial communication state-machine
 */
class FstopComms {

    // comms settings & timing
    static const int COM_BAUD=9600;
    static const unsigned long COM_TIMEOUT=400e3;
    static const unsigned long COM_CMDTIMEOUT=50e3;
    static const unsigned long COM_LCDTIMEOUT=1e6;
    static const unsigned long COM_PERIOD=100e3;

    // communication protocol bytes
    static const char COM_KEEPALIVE=0x80;
    static const char COM_READ=0x81;
    static const char COM_WRITE=0x82;
    static const char COM_LIGHT=0x83;
    static const char COM_READACK=0x91;
    static const char COM_WRITEACK=0x92;
    static const char COM_LIGHTACK=0x93;
    static const char COM_NAK=0x9F;
    static const char COM_CHKFAIL=0x9E;

    static const int PKT_CMD=0;
    static const int PKT_LEN=1;
    static const int PKT_ADDR=2;

    static const int PKT_MAXREQ=64;
    static const int PKT_SHORTHDR=4;// cmd, len, addr*2
    static const int PKT_HEADER=5;  // cmd, len, addr*2, checksum
    static const int PKT_BUFFER=PKT_MAXREQ+PKT_HEADER;

    static const int EEPROM_MIN_READ=0x0000;
    static const int EEPROM_MIN_WRITE=0x0000;
    static const int EEPROM_MAX_READ=EE_TOP;
    static const int EEPROM_MAX_WRITE=EE_TOP;

    static const char *BAD_WRITE;
    static const char *BAD_READ;
    static const char *BAD_LIGHT;
    static const char *CONNECTED;
    static const char *CHECKSUM_FAIL;

public:

    FstopComms(LiquidCrystal &l, OpenExecutor &oe);

    /// initialise port
    void begin();
    /// reset communication state-machine
    void reset();
    /// inspect port and talk
    /// @return true if connection closed
    bool poll();

private:

    /// char received in poll()
    void rx(char ch);
    /// send a char
    void tx(char ch);
    /// send buffer
    void txCmd();
    /// reject request and reset state machine but no disconnect
    void nak(const char *s);
    /// show error message
    void error(const char *s);

    /// compute xor of cmd[0..buflen-1];
    char checksum();
    /// validate checksum and reset or disconnect
    /// @return true if checksum is OK
    bool checkcheck();

    /// get address (2-byte) from packet
    unsigned int getAddr();

    void respondRead();
    void respondWrite();
    void respondLight();

    LiquidCrystal &disp;
    OpenExecutor &exec;

    unsigned long lastrx, lasttx, lastlcd;  ///< times of recent events
    bool incmd, connected;                  ///< connection state
    char cmd[PKT_BUFFER];                   ///< data buffer
    unsigned char bufwant;                  ///< how much we want in the buffer
    unsigned char buflen;                   ///< how much is in the buffer
};

/**
 * State-machine implementing fstop timer
 *
 */
class FstopTimer {
public:
    // bounds of exposure
    static const int MAXSTOP=999;
    static const int MINSTOP=-800;
    static const int INVALIDSTOP=0x7FFF;

    static const char *VERSION;
    static const char VERSIONCODE=0x06;

    enum {
        ST_SPLASH,
        ST_MAIN,
        ST_EDIT,
        ST_EDIT_EV,
        ST_EDIT_TEXT,
        ST_EXEC,
        ST_FOCUS,
        ST_IO,
        ST_IO_LOAD,
        ST_IO_SAVE,
        ST_COMMS,
        ST_TEST,
        ST_TEST_CHANGEB,
        ST_TEST_CHANGES,
        ST_CONFIG,
        ST_CONFIG_DRY,
        ST_CONFIG_ROTARY,
        ST_CONFIG_WARMUP,
        // ST_METER,
        ST_COUNT
    };

    /// constructor
    /// @param l display to write to
    /// @param k keypad to read from
    /// @param r rotary encoder for exposure changes
    /// @param b button to start/stop exposures
    /// @param p_e exposure pin (high = on)
    /// @param p_p beep pin (not used much)
    /// @param p_bl backlight pin, connect via BC548
    FstopTimer(LiquidCrystal &l, SMSKeypad &k, RotaryEncoder &r,
               ButtonDebounce &b,
               char p_e, char p_b, char p_bi);

    /// setup IO
    void begin();

    /// continue exucution of current state, ponder input
    void poll();

    /// big-endian 2-byte EEPROM IO
    static void eeWrite16(unsigned int addr, int value);
    static int eeRead16(unsigned int addr);

private:


    char inbuf[17];
    
    LiquidCrystal &disp;
    SMSKeypad &keys;
    RotaryEncoder &rotary;
    ButtonDebounce &button;
    SMSKeypad::Context smsctx;
    DecimalKeypad deckey;
    OpenExecutor openexec;
    FstopComms comms;
    DecimalKeypad::Context expctx;
    DecimalKeypad::Context stepctx;
    DecimalKeypad::Context dryctx;
    DecimalKeypad::Context intctx;

    /// programs to execute
    Program current, strip;
    /// current config for test strips
    int stripbase, stripstep;
    bool stripcover;

    Executor exec;

    /// LCD PWM factor
    int brightness;
    /// whether drydown correction is currently applied
    bool drydown_apply;
    /// paper drydown factor
    char drydown; 
    /// lamp warmup correction, milliseconds
    int warmup;
    /// exposure that is being edited
    int expnum;
    /// exposure change using rotary encoder
    int rotexp;
    /// where we're up to in a program-exec when focusing
    char focusphase;

    /// hardware pin numbers
    char pin_expose, pin_exposebtn, pin_beep, pin_backlight;

    /// lightmeter state
    char lastPWP;
    unsigned long lastMeterRead;

    /// all state-machine functions have this signature
    typedef void (FstopTimer::* voidfunc)();
    static voidfunc sm_enter[ST_COUNT];
    static voidfunc sm_poll[ST_COUNT];

    /// current state
    int curstate;
    int prevstate;

    void errorBeep();

    /// modify a value and clamp it to [MINSTOP, MAXSTOP]
    void clampExposure(int &orig, int delta);

    /**
     * Enter a new state in the state machine
     */
    void changeState(int st);

    /// invert and save the drydown-application bit
    void toggleDrydown();

    /// exec the current program
    void execCurrent();

    /// exec the test strip
    void execTest();

    /// change backlight intensity
    void setBacklight();

    /// state-machine body
    void st_splash_enter();
    void st_splash_poll();
    void st_main_enter();
    void st_main_poll();
    void st_exec_enter();
    void st_exec_poll();
    void st_focus_enter();
    void st_focus_poll();
    void st_edit_enter();
    void st_edit_poll();
    void st_edit_ev_enter();
    void st_edit_ev_poll();
    void st_edit_text_enter();
    void st_edit_text_poll();
    void st_io_enter();
    void st_io_poll();
    void st_io_load_enter();
    void st_io_load_poll();
    void st_io_save_enter();
    void st_io_save_poll();
    void st_comms_enter();
    void st_comms_poll();
    void st_test_enter();
    void st_test_poll();
    void st_test_changeb_enter();
    void st_test_changeb_poll();
    void st_test_changes_enter();
    void st_test_changes_poll();
    void st_config_enter();
    void st_config_poll();
    void st_config_dry_enter();
    void st_config_dry_poll();
    void st_config_rotary_enter();
    void st_config_rotary_poll();
    void st_config_warmup_enter();
    void st_config_warmup_poll();
    // void st_meter_enter();
    // void st_meter_poll();

    // backlight bounds
    static const char BL_MIN=0;
    static const char BL_MAX=8;


};


#endif // _FSTOP_TIMER_H_
