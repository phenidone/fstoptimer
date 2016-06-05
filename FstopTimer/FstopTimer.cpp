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


#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <Keypad.h>
#include <FstopTimer.h>

const char *FstopTimer::VERSION="F/Stop Timer ";

/// functions to exec on entering each state
FstopTimer::voidfunc FstopTimer::sm_enter[]
   ={ &FstopTimer::st_splash_enter,
      &FstopTimer::st_main_enter,
      &FstopTimer::st_edit_enter, 
      &FstopTimer::st_edit_ev_enter, 
      &FstopTimer::st_edit_text_enter, 
      &FstopTimer::st_exec_enter,
      &FstopTimer::st_focus_enter,
      &FstopTimer::st_io_enter, 
      &FstopTimer::st_io_load_enter, 
      &FstopTimer::st_io_save_enter,
      &FstopTimer::st_comms_enter,
      &FstopTimer::st_test_enter, 
      &FstopTimer::st_test_changeb_enter, 
      &FstopTimer::st_test_changes_enter, 
      &FstopTimer::st_config_enter,
      &FstopTimer::st_config_dry_enter,
      &FstopTimer::st_config_rotary_enter,
      &FstopTimer::st_config_warmup_enter
 };
/// functions to exec when polling within each state
FstopTimer::voidfunc FstopTimer::sm_poll[]
   ={ &FstopTimer::st_splash_poll, 
      &FstopTimer::st_main_poll, 
      &FstopTimer::st_edit_poll,
      &FstopTimer::st_edit_ev_poll,
      &FstopTimer::st_edit_text_poll,
      &FstopTimer::st_exec_poll, 
      &FstopTimer::st_focus_poll, 
      &FstopTimer::st_io_poll, 
      &FstopTimer::st_io_load_poll, 
      &FstopTimer::st_io_save_poll,
      &FstopTimer::st_comms_poll,
      &FstopTimer::st_test_poll,
      &FstopTimer::st_test_changeb_poll,
      &FstopTimer::st_test_changes_poll,
      &FstopTimer::st_config_poll,
      &FstopTimer::st_config_dry_poll,
      &FstopTimer::st_config_rotary_poll,
      &FstopTimer::st_config_warmup_poll
};

const char *FstopComms::BAD_WRITE=    "   Bad Write    ";
const char *FstopComms::BAD_READ=     "    Bad Read    ";
const char *FstopComms::BAD_LIGHT=    "   Bad Light    ";
const char *FstopComms::CONNECTED=    " Host Connected ";
const char *FstopComms::CHECKSUM_FAIL=" Checksum Fail  ";

// shared temporary string buffer for LCD updates
static char dispbuf[17];


FstopComms::FstopComms(LiquidCrystal &l, OpenExecutor &oe)
    : disp(l), exec(oe)
{
    lastlcd=lasttx=lastrx=micros();
    connected=false;
}

void FstopComms::begin()
{
    Serial.begin(COM_BAUD);
    reset();
}

void FstopComms::reset()
{
    connected=false;
    incmd=false;
    buflen=0;
    bufwant=1;
    lasttx=micros();
    exec.reset();
}

bool FstopComms::poll()
{
    while(Serial.available()){
        rx((char)Serial.read());        
    }

    unsigned long now=micros();

    // check for timeouts
    if(connected && now-lastrx > COM_TIMEOUT){
        reset();
    }
    if(incmd && now-lastrx > COM_CMDTIMEOUT){
        reset();
    }
    if(now-lasttx > COM_PERIOD)
        tx(COM_KEEPALIVE);
    if(now-lastlcd > COM_LCDTIMEOUT){
        disp.setCursor(0, 1);
        disp.print("                ");
        lastlcd=now;
    }

    // turn lights on/off as required & manage time
    exec.poll();

    // did we completely fail (timeout or bad command)
    return !connected;
}

void FstopComms::rx(char ch)
{
    lastrx=micros();

    // store byte
    if(buflen >= PKT_BUFFER)
        buflen=PKT_BUFFER-1;
    cmd[buflen++]=ch;
    incmd=true;

    // got the data we expected...
    if(buflen >= bufwant){
        if(!connected){
            connected=true;
            disp.clear();
            disp.print(CONNECTED);
            lastlcd=lastrx;
        }

        switch(cmd[0]){

            // keepalive; do nothing
        case COM_KEEPALIVE:
            bufwant=1;
            buflen=0;
            incmd=false;
            break;

            // request to read data
        case COM_READ:            
            if(bufwant == 1){
                // wait for the rest of the packet
                bufwant=PKT_HEADER;
            }
            else{
                respondRead();
            }
            break;

            // request to write data
        case COM_WRITE:
            if(bufwant == 1){
                // wait for rest of header
                bufwant=PKT_SHORTHDR;
            }
            else if(bufwant == PKT_SHORTHDR){
                // got length, decide how much packet to wait for
                char len=cmd[PKT_LEN];
                if(len < 1 || len > PKT_MAXREQ){
                    nak(BAD_WRITE);
                }
                else{
                    bufwant=PKT_HEADER+len;  // header+data+checksum
                }
            }
            else{
                // got whole packet
                respondWrite();
            }
            break;

            // turn the output on
        case COM_LIGHT:
            if(bufwant == 1){
                bufwant=PKT_HEADER;  // cmd, on/off, duration, checksum
            }
            else{
                respondLight();
            }
            break;            

            // disconnect
        default:
            reset();
        }
    }
}

void FstopComms::respondLight()
{
    char on=cmd[PKT_LEN];
    int duration=(int) getAddr();       // abusing signedness here

    if(on < 0 || on > 1 || duration < FstopTimer::MINSTOP || duration > FstopTimer::MAXSTOP){
        nak(BAD_LIGHT);
    }

    if(on == 0){
        exec.reset();
    }
    else{
        exec.setStops(duration);
    }

    cmd[PKT_CMD]=COM_LIGHTACK;
    buflen=PKT_SHORTHDR;
    txCmd();
}

void FstopComms::respondRead()
{
    if(!checkcheck())
        return;

    unsigned int addr=getAddr();
    char len=cmd[PKT_LEN];

    if(len < 1 || len > PKT_MAXREQ || addr < EEPROM_MIN_READ || addr+len > EEPROM_MAX_READ){
        nak(BAD_READ);
        return;
    }

    // read from EEPROM into buffer
    cmd[PKT_CMD]=COM_READACK;

    for(char i=0;i<len;++i){
        cmd[i+PKT_SHORTHDR]=EEPROM.read(addr++);
    }
    buflen=len+PKT_SHORTHDR;

    // append checksum and send.
    txCmd();
}

void FstopComms::respondWrite()
{
    if(!checkcheck())
        return;

    unsigned int addr=getAddr();
    char len=cmd[PKT_LEN];

    if(len < 1 || len > PKT_MAXREQ || addr < EEPROM_MIN_WRITE || addr+len > EEPROM_MAX_WRITE){
        nak(BAD_WRITE);
        return;
    }

    // copy from buffer to EEPROM
    char bp=PKT_SHORTHDR;
    for(char i=0;i<len;++i){
        EEPROM.write(addr++, cmd[bp++]);
    }

    cmd[PKT_CMD]=COM_WRITEACK;
    buflen=PKT_SHORTHDR;

    // recompute checksum, send without data
    txCmd();
}

unsigned int FstopComms::getAddr()
{
    unsigned char c1=cmd[PKT_ADDR], c0=cmd[PKT_ADDR+1];   // big-endian
    return (int(c1)<<8) + int(c0);
}

bool FstopComms::checkcheck()
{
    char chk=checksum();

    incmd=false;
    buflen=0;
    bufwant=1;

    if(chk != 0){
        error(CHECKSUM_FAIL);
        tx(COM_CHKFAIL);
        return false;
    }
    return true;
}

char FstopComms::checksum()
{
    char sum=0;
    for(int i=0;i<buflen;++i)
        sum ^= cmd[i];

    return sum;
}


void FstopComms::tx(char ch)
{
    lasttx=micros();
    Serial.write(ch);
}

void FstopComms::txCmd()
{
    // compute & append checksum if there is room
    if(buflen < PKT_BUFFER){
        cmd[buflen++]=checksum();
    }

    // transmit
    for(int i=0;i<buflen;++i){
        Serial.write(cmd[i]);
    }
    lasttx=micros();

    incmd=false;
    buflen=0;
    bufwant=1;
}

void FstopComms::nak(const char *s)
{
    error(s);
    tx(COM_NAK);
    incmd=false;
    buflen=0;
    bufwant=1;
}

void FstopComms::error(const char *s)
{
    disp.setCursor(0, 1);
    disp.print(s);
    lastlcd=micros();
}


FstopTimer::FstopTimer(LiquidCrystal &l, SMSKeypad &k, RotaryEncoder &r,
                       ButtonDebounce &b,
                       char p_e, char p_b, char p_bl)
    : disp(l), keys(k), rotary(r), button(b),
      smsctx(&inbuf[0], 14, &disp, 0, 0),
      deckey(keys), openexec(l, p_e, warmup),
      comms(l, openexec),
      expctx(&inbuf[0], 1, 2, &disp, 0, 1, true),
      stepctx(&inbuf[0], 1, 2, &disp, 0, 1, false),
      dryctx(&inbuf[0], 0, 2, &disp, 0, 1, false),
      intctx(&inbuf[0], 1, 0, &disp, 0, 1, false),
      exec(l, keys, button, p_e, warmup),
      pin_expose(p_e), 
      pin_beep(p_b), pin_backlight(p_bl)
{

    // init libraries
    prevstate=curstate=ST_MAIN;
    focusphase=-1;
}

void FstopTimer::setBacklight()
{
    brightness=EEPROM.read(EE_BACKLIGHT);
    if(brightness > BL_MAX)
        brightness=BL_MAX;
    if(brightness < BL_MIN)
        brightness=BL_MIN;

    analogWrite(pin_backlight, (1<<brightness)-1);
}

void FstopTimer::begin()
{
    // init raw IO
    pinMode(pin_expose, OUTPUT);
    // pinMode(pin_beep, OUTPUT);
    pinMode(pin_backlight, OUTPUT);
    pinMode(pin_exposebtn, INPUT);

    digitalWrite(pin_expose, LOW);

    // load & apply backlight settings
    setBacklight();
   
    drydown=EEPROM.read(EE_DRYDOWN);
    drydown_apply=EEPROM.read(EE_DRYAPPLY);
    warmup=eeRead16(EE_WARMUP);

    current.clear();
    stripbase=eeRead16(EE_STRIPBASE);
    stripstep=eeRead16(EE_STRIPSTEP);
    stripcover=EEPROM.read(EE_STRIPCOV);

    // prevent client-overwrite shenanigans
    EEPROM.write(EE_VERSION, VERSIONCODE);

    rotexp=EEPROM.read(EE_ROTARY);

    comms.begin();
    exec.begin();

    // boot the state machine
    changeState(ST_SPLASH);
}

void FstopTimer::eeWrite16(unsigned int addr, int value)
{
    EEPROM.write(addr, (value>>8) & 0xFF);
    EEPROM.write(addr+1, value & 0xFF);
}

int FstopTimer::eeRead16(unsigned int addr)
{
    return (EEPROM.read(addr)<<8) | EEPROM.read(addr+1);
}

void FstopTimer::toggleDrydown()
{
    drydown_apply=!drydown_apply;
    EEPROM.write(EE_DRYAPPLY, drydown_apply);
}

void FstopTimer::st_splash_enter()
{
    disp.clear();
    disp.print(VERSION);
    dtostrf(EEPROM.read(EE_VERSION)*0.1, 3, 1, dispbuf);
    disp.print(dispbuf);
    disp.setCursor(0, 1);
    disp.print("W Brodie-Tyrrell");
}

void FstopTimer::st_splash_poll()
{
    if(Serial.available()){
        changeState(ST_COMMS);
    }
    if(keys.available()){
        keys.readAscii();
        changeState(ST_MAIN);
    }
}

/// begin the main-menu state; write to display
void FstopTimer::st_main_enter()
{
    disp.clear();
    disp.setCursor(0,0);
    disp.print("A:Edit   B: IO");
    disp.setCursor(0,1);
    disp.print("C:Config D: Test");
}

/// polling function executed while in the Main state
void FstopTimer::st_main_poll()
{   
    if(button.hadPress()){
        changeState(ST_FOCUS);
        return;
    }
    if(keys.available()){
        char ch=keys.readAscii();
        if(isspace(ch))
            return;
        switch(ch){
        case 'A':
            // exec.current.clear();
            changeState(ST_EDIT);
            break;
        case 'B':
            changeState(ST_IO);        
            break;
        case 'C':
            changeState(ST_CONFIG);
            break;
        case 'D':
            changeState(ST_TEST);
            break;
        case '#':
            execCurrent();
            break;
        case '*':
            changeState(ST_FOCUS);
            break;
        default:
            break;
            // errorBeep();
        }
    }
    if(Serial.available()){
        changeState(ST_COMMS);
    }   
}

void FstopTimer::execCurrent()
{
    if(!current.compile(drydown_apply ? drydown : 0, warmup)){
        disp.clear();
        disp.print("  Cannot Print");
        disp.setCursor(0, 1);
        disp.print(" Dodges > Base");
        exec.setProgram(NULL);
        // errorBeep();
        delay(2000);
        changeState(ST_EDIT);
    }
    else{
        exec.setProgram(&current);
        changeState(ST_EXEC);
    }
}

void FstopTimer::st_exec_enter()
{
    Program *p=exec.getProgram();
    // we assume it compiles if we're in this state
    p->compile(drydown_apply ? drydown : 0, warmup);
    disp.clear();
    exec.setDrydown(drydown_apply);

    if(focusphase >= 0){
        exec.changePhase(focusphase);
    }
    focusphase=-1;
}

void FstopTimer::st_exec_poll()
{
    if(button.hadPress()){
        exec.expose();
        return;
    }   

    if(keys.available()){
        char ch=keys.readAscii();
        if(isspace(ch))
            return;
        switch(ch){
        case '#':
            // perform exposure!
            exec.expose();
            break;
        case '*':
            // focus
            focusphase=exec.getPhase();
            changeState(ST_FOCUS);
            break;
        case 'A':
            // abort/restart
            disp.clear();
            disp.print("Restart Exposure");
            delay(1000);
            exec.changePhase(0);
            break;
        case 'B':
            // skip exposure
            exec.nextPhase();
            break;
        case 'C':
            // main menu
            changeState(ST_MAIN);
            break;
        case 'D':
            // toggle drydown & recompile
            toggleDrydown();
            if(exec.getPhase() != 0){
                disp.clear();
                disp.print("Restart Exposure");
                disp.setCursor(0,1);
                disp.print("For Drydown Chg");
                delay(1000);
            }
            // forces recompile -> apply drydown
            changeState(ST_EXEC);
            break;
        } 
    }
}

void FstopTimer::st_focus_enter()
{
    disp.clear();
    disp.print("     Focus!");    
    digitalWrite(pin_expose, HIGH);

    PeriodMeasure::setup();
    lastMeterRead=millis();
    lastPWP=PeriodMeasure::getPeriodCount();
}

void FstopTimer::st_focus_poll()
{
    // if we got meter a reading, show it
    char pwp=PeriodMeasure::getPeriodCount();
    unsigned long now=millis();
    if((unsigned char)(pwp-lastPWP) >= 8 && now - lastMeterRead > 100){
        lastPWP=pwp;

        // system clock is 16e6, therefore this gives us freq/10Hz
        float freq=16e5/PeriodMeasure::readSmoothed();
        // so 0 stops is 10Hz
        float stops=log(freq)/log(2.0f);

        disp.setCursor(0, 1);
        dtostrf(stops, 0, 2, dispbuf);
        disp.print(dispbuf);
        disp.print(" stops  ");
        lastMeterRead=now;
    }
    else if(now - lastMeterRead > 2000){
        disp.setCursor(0, 1);
        disp.print("   NO SENSOR    ");
        lastMeterRead=now;
    }

    bool done=button.hadPress();

    // return to prev state?
    if(keys.available()){
        keys.readRaw();
        done=true;
    }

    if(done){
        digitalWrite(pin_expose, LOW);
        changeState(prevstate);
    }
}

void FstopTimer::st_edit_enter()
{
    // don't print help
/*
    disp.clear();
    disp.setCursor(0,0);
    disp.print("Edit Program");
    disp.setCursor(0,1);
    disp.print("A:TX B:EV n:SLOT");
    delay(1000);
*/

    rotary.getDelta();       // clear any accum'd motion
    expnum=0;
    current[expnum].display(disp, false);
}

void FstopTimer::st_edit_poll()
{
    // keypad events?
    if(keys.available()){
        char ch=keys.readAscii();
        if(isspace(ch))
            return;
        switch(ch){
        case 'A':
            // edit text
            changeState(ST_EDIT_TEXT);
            break;
        case 'B':
            // edit EV
            changeState(ST_EDIT_EV);
            break;
        case 'C':
        case 'D':
            changeState(ST_MAIN);
            break;
        case '#':
            execCurrent();
            break;
        case '*':
            changeState(ST_FOCUS);
            break;
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
            // change to different step for editing
            expnum=ch-'1';
            current[expnum].display(disp, false);
            break;
        default:
            errorBeep();
        }
    }

    // knob events?
    int rot=rotary.getDelta();
    if(rot != 0){
        clampExposure(current[expnum].stops, rot*rotexp);
        current[expnum].display(disp, false);
    }
}

void FstopTimer::st_edit_ev_enter()
{
    deckey.setContext(&expctx);
}

void FstopTimer::st_edit_ev_poll()
{
    if(deckey.poll()){
        if(expctx.exitcode != Keypad::KP_C){
            current[expnum].stops=expctx.result;
        }
        current[expnum].display(disp, false);
        // sly state change without expnum=0
        curstate=ST_EDIT;
    }
}

void FstopTimer::st_edit_text_enter()
{
    keys.setContext(&smsctx);
}

void FstopTimer::st_edit_text_poll()
{
    if(keys.poll()){
        if(smsctx.exitcode != Keypad::KP_C){
            strcpy(current[expnum].text, smsctx.buffer);
        }
        current[expnum].display(disp, false);
        curstate=ST_EDIT;        
    }
}

void FstopTimer::st_io_enter()
{
    disp.clear();
    disp.print("A: New  B: Load");
    disp.setCursor(0, 1);
    disp.print("C: Save D: Main");
}

void FstopTimer::st_io_poll()
{
    if(keys.available()){
        char ch=keys.readAscii();
        switch(ch){
        case 'A':
            current.clear();
            changeState(ST_EDIT);
            break;
        case 'B':
            changeState(ST_IO_LOAD);
            break;
        case 'C':
            changeState(ST_IO_SAVE);
            break;
        case 'D':
            changeState(ST_MAIN);
            break;
        default:
            errorBeep();
        }
    }
}

void FstopTimer::st_io_load_enter()
{
    disp.clear();
    disp.print("Select Load Slot");
    deckey.setContext(&intctx);
}

void FstopTimer::st_io_load_poll()
{
    if(deckey.poll()){
        disp.clear();
        char slot=intctx.result;

        if(current.load(slot)){ 
            disp.print("Program Loaded");
        }
        else{
            disp.print("Slot not in 1..7");
            errorBeep();
        }
        delay(1000);

        changeState(ST_EDIT);
    }
}

void FstopTimer::st_io_save_enter()
{
    disp.clear();
    disp.print("Select Save Slot");
    deckey.setContext(&intctx);
}

void FstopTimer::st_io_save_poll()
{
    if(deckey.poll()){
        disp.clear();
        char slot=intctx.result;
        if(slot >= Program::FIRSTSLOT && slot <= Program::LASTSLOT){
            current.save(slot);
            disp.print("Program Saved");
        }
        else{
            disp.print("Slot not in 1..7");
            errorBeep();
        }
        delay(1000);

        changeState(ST_MAIN);
    }
}

void FstopTimer::st_comms_enter()
{
    comms.reset();
}

void FstopTimer::st_comms_poll()
{
    if(comms.poll()){
        comms.reset();
        changeState(ST_MAIN);
    }
}

void FstopTimer::st_test_enter()
{
    disp.clear();
    disp.print("A:");
    disp.print(stripcover ? "Cover" : "Indiv");
    disp.print(" B:Change");
    disp.setCursor(0, 1);
    dtostrf(0.01f*stripbase, 0, 2, dispbuf);
    disp.print(dispbuf);
    disp.print(" by ");
    dtostrf(0.01f*stripstep, 0, 2, dispbuf);
    disp.print(dispbuf);
    disp.setCursor(15, 1);
    disp.print(drydown_apply ? "D" : " ");
    rotary.getDelta();
}

void FstopTimer::st_test_poll()
{
    bool go=button.hadPress();

    if(keys.available()){
        char ch=keys.readAscii();
        switch(ch){
        case 'A':
            // toggle type
            stripcover=!stripcover;
            EEPROM.write(EE_STRIPCOV, stripcover);
            changeState(ST_TEST);
            break;
        case 'B':
            // change exposures
            changeState(ST_TEST_CHANGEB);
            break;
        case 'C':
            // main menu
            changeState(ST_MAIN);
            break;
        case 'D':
            // toggle drydown
            toggleDrydown();
            changeState(ST_TEST);
            break;
        case '#':
            // perform exposure
            go=true;
            break;
        }
    }

    int rot=rotary.getDelta();
    if(rot != 0){
        clampExposure(stripbase, rot*rotexp);
        changeState(ST_TEST);
    }

    if(go){
        strip.configureStrip(stripbase, stripstep, stripcover);
        exec.setProgram(&strip);
        changeState(ST_EXEC);
    }
}

void FstopTimer::st_test_changeb_enter()
{
    disp.clear();
    disp.print("Test Strip Base:");
    deckey.setContext(&expctx);
}

void FstopTimer::st_test_changeb_poll()
{
    if(deckey.poll()){
        if(expctx.exitcode != Keypad::KP_C){
            stripbase=expctx.result;
            eeWrite16(EE_STRIPBASE, stripbase);
        }
        changeState(ST_TEST_CHANGES);
    }
}

void FstopTimer::st_test_changes_enter()
{
    disp.clear();
    disp.print("Test Strip Step:");
    deckey.setContext(&stepctx);
}

void FstopTimer::st_test_changes_poll()
{
    if(deckey.poll()){
        if(stepctx.exitcode != Keypad::KP_C){
            stripstep=stepctx.result;
            eeWrite16(EE_STRIPSTEP, stripstep);
        }
        changeState(ST_TEST);
    }
}

void FstopTimer::st_config_enter()
{
    disp.clear();
    disp.setCursor(0,0);
    disp.print("A:Rotary C:Warm ");
    disp.setCursor(0,1);
    disp.print("B:Brite  D:Drydn");  
}

void FstopTimer::st_config_poll()
{
    if(keys.available()){
        char ch=keys.readAscii();
        switch(ch){
        case 'A':
            changeState(ST_CONFIG_ROTARY);
            break;
        case 'B': 
            // change brightness
            ++brightness;
            if(brightness > BL_MAX)
                brightness=BL_MIN;
            EEPROM.write(EE_BACKLIGHT, brightness);
            setBacklight();
            break;
            // change warmup
        case 'C':
            changeState(ST_CONFIG_WARMUP);
            break;
        case 'D':
            // change drydown
            changeState(ST_CONFIG_DRY);
            break;
        default:
            // main menu
            changeState(ST_MAIN);
        }
    }
}

void FstopTimer::st_config_dry_enter()
{
    disp.clear();
    disp.print("Drydown Factor:");
    deckey.setContext(&dryctx);
}

void FstopTimer::st_config_dry_poll()
{
    if(deckey.poll()){
        disp.clear();
        if(dryctx.exitcode == Keypad::KP_C){
            disp.print("Cancelled");
        }
        else{
            drydown=abs(dryctx.result);
            EEPROM.write(EE_DRYDOWN, drydown);
            disp.print("Changed");
        }
        disp.setCursor(0, 1);
        disp.print("Drydown = ");
        dtostrf(0.01f*drydown, 0, 2, dispbuf);
        disp.print(dispbuf);
        delay(1000);
    
        changeState(ST_MAIN);
    }
}

void FstopTimer::st_config_warmup_enter()
{
    disp.clear();
    disp.print("Warmup Time, s:");
    deckey.setContext(&dryctx);
}

void FstopTimer::st_config_warmup_poll()
{
    if(deckey.poll()){
        disp.clear();
        if(dryctx.exitcode == Keypad::KP_C){
            disp.print("Cancelled");
        }
        else{
            warmup=abs(dryctx.result)*10;
            eeWrite16(EE_WARMUP, warmup);
            disp.print("Changed");
        }
        disp.setCursor(0, 1);
        disp.print("Warmup = ");
        dtostrf(0.001f*warmup, 0, 2, dispbuf);
        disp.print(dispbuf);
        disp.print("s");
        delay(1000);
    
        changeState(ST_MAIN);
    }
}

void FstopTimer::st_config_rotary_enter()
{
    disp.clear();
    disp.print("Rotary Step:");
    deckey.setContext(&dryctx);
}

void FstopTimer::st_config_rotary_poll()
{
    if(deckey.poll()){
        disp.clear();
        if(dryctx.exitcode == Keypad::KP_C){
            disp.print("Cancelled");
        }
        else{
            rotexp=abs(dryctx.result);
            EEPROM.write(EE_ROTARY, rotexp);
            disp.print("Changed");
        }
        disp.setCursor(0, 1);
        disp.print("Step = ");
        dtostrf(0.01f*rotexp, 0, 2, dispbuf);
        disp.print(dispbuf);
        delay(1000);
    
        changeState(ST_MAIN);
    }
}

/*
void FstopTimer::st_meter_enter()
{
    PeriodMeasure::setup();
    lastMeterRead=millis();
    lastPWP=PeriodMeasure::getPeriodCount();
    disp.clear();
    disp.print("  Light Meter");
}

void FstopTimer::st_meter_poll()
{
    // any key to return to main menu
    if(keys.available()){
        char ch=keys.readAscii();
        PeriodMeasure::stop();
        changeState(ST_MAIN);
        return;
    }

    // if we got a reading, show it
    char pwp=PeriodMeasure::getPeriodCount();
    unsigned long now=millis();
    if((unsigned char)(pwp-lastPWP) >= 8){
        lastPWP=pwp;

        // system clock is 16e6, therefore this gives us freq/10Hz
        float freq=16e5/PeriodMeasure::readSmoothed();
        // so 0 stops is 10Hz
        float stops=log(freq)/log(2.0f);

        disp.setCursor(0, 1);
        dtostrf(stops, 0, 2, dispbuf);
        disp.print(dispbuf);
        disp.print(" stops  ");
        lastMeterRead=now;
    }
    else if(now - lastMeterRead > 1000){
        disp.setCursor(0, 1);
        disp.print("   NO SENSOR    ");
        lastMeterRead=now;
    }
}
*/

void FstopTimer::changeState(int st)
{
    prevstate=curstate;
    curstate=st;
    button.hadPress();  // clear any press that might interfere in next state
    (this->*sm_enter[curstate])();
}


void FstopTimer::poll()
{
    // scan keypad
    keys.scan();
    button.scan();
  
    // attend to whatever the state requires
    (this->*sm_poll[curstate])(); 
}

void FstopTimer::clampExposure(int &expos, int delta)
{
    expos+=delta;

    if(expos > MAXSTOP)
        expos=MAXSTOP;
    if(expos < MINSTOP)
        expos=MINSTOP;
}

void Program::clear()
{
    // base
    exposures[0].stops=200;
    strcpy(exposures[0].text, "Base Exposure");
    isstrip=false;
   
    // invalid
    for(int i=1;i<MAXEXP;++i){
        exposures[i].stops=0;
        strcpy(exposures[i].text, "Undefined");
    }
}

void Program::configureStrip(int base, int step, bool cov)
{
    isstrip=true;
    cover=cov;

    int expos=base;
    for(char i=0;i<MAXEXP;++i){
        exposures[i].stops=expos;
        strcpy(exposures[i].text, "Strip ");
        dtostrf(0.01f*expos, 1, 2, &exposures[i].text[6]);
        strcpy(&exposures[i].text[10], cov ? " Cov" : " Ind");

        expos+=step;
    }
}

void Program::clipExposures()
{
    // don't want to be here forever or overflow the screen
    for(char i=0;i<MAXEXP;++i){
        if(exposures[i].ms > MAXMS) 
            exposures[i].ms=MAXMS;
    }
}

bool Program::compile(char dryval, int warmup)
{
    if(isstrip){
        if(cover)
            compileStripCover(dryval, warmup);
        else
            compileStripIndiv(dryval, warmup);

        clipExposures();
        return true;
    }
    else{
        return compileNormal(dryval, warmup);
    }
}

void Program::compileStripIndiv(char dryval, int warmup)
{
    for(int i=0;i<MAXEXP;++i){
        exposures[i].ms=hunToMillis(exposures[i].stops-dryval)+warmup;
    }
}

void Program::compileStripCover(char dryval, int warmup)
{
    unsigned long sofar=0;

    for(int i=0;i<MAXEXP;++i){
        unsigned long thisexp=hunToMillis(exposures[i].stops-dryval);
        exposures[i].ms=thisexp-sofar+warmup;
        sofar=thisexp;
    }
}

bool Program::compileNormal(char dryval, int warmup)
{
    // base exposure, perhaps with drydown
    int base=exposures[0].stops-dryval;
      
    exposures[0].ms=hunToMillis(base);
    
    // figure out the total exposure desired for each step (base+adjustment)
    unsigned long dodgetime=0;
    for(int i=1;i<MAXEXP;++i){
        if(exposures[i].stops == 0){
            exposures[i].ms=0;
            continue;
        }

        exposures[i].ms=hunToMillis(base+exposures[i].stops);


        // is dodge?
        if(exposures[i].stops < 0){
            // convert to time-difference
            exposures[i].ms=exposures[0].ms-exposures[i].ms;
            // keep track of total time spent dodging          
            dodgetime+=exposures[i].ms;  
        }
        else{
            // convert to time-difference
            exposures[i].ms-=exposures[0].ms;          
        }
    }
    
    // fail if we have more dodge than base exposure
    if(dodgetime > exposures[0].ms)
        return false;
    
    // take off the dodgetime
    exposures[0].ms-=dodgetime;
    
    // apply warmup correction post-hoc
    for(int i=0;i<MAXEXP;++i){
        if(exposures[i].ms > 0)
            exposures[i].ms += warmup;
    }

    clipExposures();

    return true;
}

void Program::Exposure::display(LiquidCrystal &disp, bool lin)
{
    // print text
    disp.clear();
    disp.setCursor(0,0);
    disp.print(text);
   
    displayTime(disp, lin);
}

void Program::Exposure::displayTime(LiquidCrystal &disp, bool lin)
{
    disp.setCursor(0,1);

    // print stops
    char used=0;
    if(stops >= 0){
        disp.print("+");
        ++used;
    }
    dtostrf(0.01f*stops, 0, 2, dispbuf);
    used+=strlen(dispbuf);
    disp.print(dispbuf);

    // print compiled seconds
    if(lin){
        disp.print("=");
        dtostrf(0.001f*ms, 0, 3, dispbuf);
        used+=strlen(dispbuf)+2;
        disp.print(dispbuf);
        disp.print("s");
     
        // fill out to 15 chars with spaces
        // keep the 16th for 'D' drydown-indicator
        for(int i=0;i<15-used;++i)
            dispbuf[i]=' ';
        dispbuf[15-used]='\0';
        disp.print(dispbuf);
    }  
}

Program::Exposure &Program::operator[](int which)
{
    return exposures[which]; 
}

unsigned long Program::hunToMillis(int hunst)
{
    return lrint(1000.0f*pow(2.0f, 0.01f*hunst));
}

int Program::slotAddr(int slot)
{
    return SLOTBASE+((slot-FIRSTSLOT)<<SLOTBITS);
}

void Program::save(int slot)
{
    if(slot < FIRSTSLOT || slot > LASTSLOT)
        return;
    
    int addr=slotAddr(slot);
    for(int i=0;i<MAXEXP;++i){
        FstopTimer::eeWrite16(addr, exposures[i].stops);
        addr+=2;
        for(int t=0;t<TEXTLEN;++t){
            EEPROM.write(addr++, exposures[i].text[t]); 
        }
    }
}

bool Program::load(int slot)
{
    if(slot < FIRSTSLOT || slot > LASTSLOT)
        return false;
    
    int addr=slotAddr(slot);
    int tmp;
    for(int i=0;i<MAXEXP;++i){
        exposures[i].stops=FstopTimer::eeRead16(addr);
        addr+=2;
        for(int t=0;t<TEXTLEN;++t){
            exposures[i].text[t]=EEPROM.read(addr++); 
        }
     
        // terminate string in memory
        exposures[i].text[TEXTLEN]='\0';
    }
    return true;
}


void FstopTimer::errorBeep()
{
    // tone(pin_beep, 2000, 100); 
}

Executor::Executor(LiquidCrystal &l, Keypad &k, ButtonDebounce &b, char p_e, int &w)
    : disp(l), keys(k), button(b), pin_expose(p_e), warmup(w)
{
    current=NULL;
}

void Executor::begin()
{
    execphase=0;
    dd=false;
}

void Executor::setProgram(Program *p)
{
    current=p;
    changePhase(0);
}


void Executor::setDrydown(bool d)
{
    dd=d;
    changePhase(0);
}

/// specify that program is up to a particular exposure; display it
void Executor::changePhase(unsigned char ph)
{
    execphase=ph;

    if(NULL == current)
        return;

    (*current)[ph].display(disp, true);
    disp.setCursor(15, 1);
    disp.print(dd ? "D" : " ");
}

/// perform a measured exposure, updating display with time remaining
/// normal input processing suspended here for a busy-loop, we're only
/// looking at time and the Expose-button for cancellation
void Executor::expose()
{
    if(NULL == current)
        return;

    // backup the duration; it will get overwritten for display purposes
    Program::Exposure &expo=(*current)[execphase];
    unsigned long msbackup=expo.ms;    // for restoring expo.ms at end
    unsigned long mstotal=msbackup;    // aimed-for total duration, pause+warmup=>increases
    unsigned long start=micros();      // when exposure began
    unsigned long now=start, dt=0, lastupdate=start;

    // begin
    digitalWrite(pin_expose, HIGH);
  
    // polling loop
    bool cancelled=false, skipped=false;
    while(!skipped){
        now=micros();
        dt=(now-start)/1000;    // works with overflows

        // time is elapsed
        if(dt >= mstotal){
            break; 
        }
    
        // time for some IO?  otherwise tighten the loop a bit
        if((mstotal-dt) > 50){
            if((now-lastupdate) > UPDATE_PERIOD){
                // re-display with reduced time remaining
                expo.ms=mstotal-dt; 
                expo.displayTime(disp, true);
                lastupdate=now;
            }

            // pause!
            button.scan();
            keys.scan();
            bool press=button.hadPress();

            if(press || (keys.available() && keys.readRaw() == Keypad::KP_HASH)){
                digitalWrite(pin_expose, LOW);
                unsigned long pausestart=micros();
                press=false;

                // insert a warmup-time & update display
                mstotal+=warmup;
                expo.ms=mstotal-dt;
                expo.displayTime(disp, true);

                // wait for keypress or button press
                // cancel on anything but Expose button
                char ch=0;
                while(true){
                    keys.scan();
                    button.scan();
                    
                    if(button.hadPress()){
                        ch=Keypad::KP_HASH;   // pretend
                        break;
                    }
                    if(keys.available()){
                        ch=keys.readRaw();
                        break;
                    }
                }
                
                switch(ch){
                case Keypad::KP_HASH:
                    // adjust clock and resume exposing
                    start+=micros()-pausestart;
                    digitalWrite(pin_expose, HIGH);
                    break;
                case Keypad::KP_B:
                    // halt and skip this exposure
                    skipped=true;
                    break;
                default:
                    // halt and cancel
                    skipped=true;
                    cancelled=true;
                }
            }
        }
    }
  
    // cease
    digitalWrite(pin_expose, LOW);
  
    // restore
    expo.ms=msbackup;
  
    if(cancelled){
        // tell user to go home
        disp.clear();
        disp.print("Prog Cancelled");
        delay(1000);
        changePhase(0);
    }   
    else{
        nextPhase();
    }
}

void Executor::nextPhase()
{
    // decide on next exposure or reset to beginning
    for(int newphase=execphase+1;newphase<Program::MAXEXP;++newphase){
        if((*current)[newphase].stops != 0){
            changePhase(newphase);
            return;
        }
    }

    disp.clear();
    disp.print("Program Complete");
    delay(1000);
    changePhase(0);
}

OpenExecutor::OpenExecutor(LiquidCrystal &l, char p_e, int &w)
    : disp(l), pin_expose(p_e), warmup(w)
{
    reset();
}

void OpenExecutor::setStops(int stops)
{
    expo.stops=stops;
    expo.ms=Program::hunToMillis(stops)+warmup;
    expo.text[0]='\0';
    mstotal=0;
    complete=false;
    on=false;
}

void OpenExecutor::reset()
{
    digitalWrite(pin_expose, LOW);
    expo.ms=0;
    complete=true;
    on=false;
}

void OpenExecutor::poll()
{
    unsigned long now=micros();

    if(complete){
        return;
    }

    // begin!
    if(expo.ms > mstotal){
        mstotal=expo.ms;
        start=now;
        lastupdate=now;
        on=true;
        digitalWrite(pin_expose, HIGH);
        return;
    }

    // check remaining time
    long dt=(now-start)/1000;
    if(dt > mstotal){
        reset();
        expo.displayTime(disp, true);
        return;
    }

    // if there is time...
    if(mstotal-dt > 50){
        // update display
        if(now-lastupdate > Executor::UPDATE_PERIOD){
            expo.ms=mstotal-dt;
            expo.displayTime(disp, true);
            lastupdate=now;
        }
    }

}
