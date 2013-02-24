/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *   Mupen64plus-input-android - plugin.c                                  *
 *   Mupen64Plus homepage: http://code.google.com/p/mupen64plus/           *
 *   Copyright (C) 2008-2011 Richard Goedeken                              *
 *   Copyright (C) 2008 Tillin9                                            *
 *   Copyright (C) 2002 Blight                                             *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.          *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define M64P_PLUGIN_PROTOTYPES 1
#include "m64p_types.h"
#include "m64p_plugin.h"
#include "m64p_common.h"
#include "m64p_config.h"

#include "plugin.h"
#include "version.h"

#ifdef __linux__
#include <unistd.h>
#include <dirent.h>
#include <fcntl.h>
#include <sys/types.h>
#include <linux/input.h>
#endif /* __linux__ */

#include <errno.h>
#include <jni.h>
#include <android/log.h>
#define printf(...) __android_log_print(ANDROID_LOG_VERBOSE, "input-android, plugin.c", __VA_ARGS__)

/* defines for the force feedback rumble support */
#ifdef __linux__
#define BITS_PER_LONG (sizeof(long) * 8)
#define OFF(x)  ((x)%BITS_PER_LONG)
#define BIT(x)  (1UL<<OFF(x))
#define LONG(x) ((x)/BITS_PER_LONG)
#define test_bit(bit, array)    ((array[LONG(bit)] >> OFF(bit)) & 1)
#endif //__linux__

/* global data definitions */
SController controller[4];   // 4 controllers

/* static data definitions */
static void (*l_DebugCallback)(void *, int, const char *) = NULL;
static void *l_DebugCallContext = NULL;
static int l_PluginInit = 0;

static unsigned short button_bits[] = {
    0x0001,  // R_DPAD
    0x0002,  // L_DPAD
    0x0004,  // D_DPAD
    0x0008,  // U_DPAD
    0x0010,  // START_BUTTON
    0x0020,  // Z_TRIG
    0x0040,  // B_BUTTON
    0x0080,  // A_BUTTON
    0x0100,  // R_CBUTTON
    0x0200,  // L_CBUTTON
    0x0400,  // D_CBUTTON
    0x0800,  // U_CBUTTON
    0x1000,  // R_TRIG
    0x2000,  // L_TRIG
    0x4000,  // Mempak switch
    0x8000   // Rumblepak switch
};

static int romopen = 0;         // is a rom opened

#ifdef __linux__
static struct ff_effect ffeffect[3];
static struct ff_effect ffstrong[3];
static struct ff_effect ffweak[3];
#endif //__linux__

//// paulscode, for the phone vibrator:
extern void Android_JNI_Vibrate( int active );

//// paulscode, for the virtual gamepad:
unsigned char virtualGamePadButtonState[4][16];
signed char virtualGamePadAxisState[4][2];
JNIEXPORT void JNICALL Java_paulscode_android_mupen64plusae_CoreInterfaceNative_updateVirtualGamePadStates(
                                    JNIEnv* env, jclass jcls, jint controllerNum,
                                    jbooleanArray mp64pButtons, jint mp64pXAxis, jint mp64pYAxis )
{
    jboolean *elements = (*env)->GetBooleanArrayElements( env, mp64pButtons, NULL );
    int b;

    for( b = 0; b < 16; b++ )
    {
        virtualGamePadButtonState[controllerNum][b] = elements[b];
    }
    (*env)->ReleaseBooleanArrayElements( env, mp64pButtons, elements, 0 );

    // from the N64 func ref: The 3D Stick data is of type signed char and in
    // the range between 80 and -80. (32768 / 409 = ~80.1)
    virtualGamePadAxisState[controllerNum][0] = (signed char) ((int) mp64pXAxis);
    virtualGamePadAxisState[controllerNum][1] = (signed char) ((int) mp64pYAxis);
}
////

/* Global functions */
void DebugMessage(int level, const char *message, ...)
{
  char msgbuf[1024];
  va_list args;

  if (l_DebugCallback == NULL)
      return;

  va_start(args, message);
  vsprintf(msgbuf, message, args);

  (*l_DebugCallback)(l_DebugCallContext, level, msgbuf);

  va_end(args);
}

static CONTROL temp_core_controlinfo[4];

/* Mupen64Plus plugin functions */
EXPORT m64p_error CALL PluginStartup(m64p_dynlib_handle CoreLibHandle, void *Context,
                                   void (*DebugCallback)(void *, int, const char *))
{
    ptr_CoreGetAPIVersions CoreAPIVersionFunc;
    
    int i, ConfigAPIVersion, DebugAPIVersion, VidextAPIVersion;

    if (l_PluginInit)
        return M64ERR_ALREADY_INIT;

    /* first thing is to set the callback function for debug info */
    l_DebugCallback = DebugCallback;
    l_DebugCallContext = Context;

    /* reset controllers */
    memset(controller, 0, sizeof(SController) * 4);
    /* set CONTROL struct pointers to the temporary static array */
    /* this small struct is used to tell the core whether each controller is plugged in, and what type of pak is connected */
    /* we only need it so that we can call load_configuration below, to auto-config for a GUI front-end */
    for (i = 0; i < 4; i++)
    {
        controller[i].control = temp_core_controlinfo + i;
        controller[i].control->Present = 1;
    }

    l_PluginInit = 1;
    return M64ERR_SUCCESS;
}

EXPORT m64p_error CALL PluginShutdown(void)
{
    if (!l_PluginInit)
        return M64ERR_NOT_INIT;

    /* reset some local variables */
    l_DebugCallback = NULL;
    l_DebugCallContext = NULL;

    l_PluginInit = 0;
    return M64ERR_SUCCESS;
}

EXPORT m64p_error CALL PluginGetVersion(m64p_plugin_type *PluginType, int *PluginVersion, int *APIVersion, const char **PluginNamePtr, int *Capabilities)
{
    /* set version info */
    if (PluginType != NULL)
        *PluginType = M64PLUGIN_INPUT;

    if (PluginVersion != NULL)
        *PluginVersion = PLUGIN_VERSION;

    if (APIVersion != NULL)
        *APIVersion = INPUT_PLUGIN_API_VERSION;
    
    if (PluginNamePtr != NULL)
        *PluginNamePtr = PLUGIN_NAME;

    if (Capabilities != NULL)
    {
        *Capabilities = 0;
    }
                    
    return M64ERR_SUCCESS;
}

/* Helper function to handle the onscreen gamepad */
static void doVirtualGamePad()
{
    int b, c;
    for( c = 0; c < 4; c++ )
    {
        for( b = 0; b < 16; b++ )
        {
            if( virtualGamePadButtonState[c][b] )
                controller[c].buttons.Value |= button_bits[b];
        }
        // from the N64 func ref: The 3D Stick data is of type signed char and in
        // the range between 80 and -80. (32768 / 409 = ~80.1)
        if( virtualGamePadAxisState[c][0] || virtualGamePadAxisState[c][1] )
        {
            // only report the stick position if it isn't back at the center
            controller[c].buttons.X_AXIS = virtualGamePadAxisState[c][0];
            controller[c].buttons.Y_AXIS = virtualGamePadAxisState[c][1];
        }
    }
}

static unsigned char DataCRC( unsigned char *Data, int iLenght )
{
    unsigned char Remainder = Data[0];

    int iByte = 1;
    unsigned char bBit = 0;

    while( iByte <= iLenght )
    {
        int HighBit = ((Remainder & 0x80) != 0);
        Remainder = Remainder << 1;

        Remainder += ( iByte < iLenght && Data[iByte] & (0x80 >> bBit )) ? 1 : 0;

        Remainder ^= (HighBit) ? 0x85 : 0;

        bBit++;
        iByte += bBit/8;
        bBit %= 8;
    }

    return Remainder;
}

/******************************************************************
  Function: ControllerCommand
  Purpose:  To process the raw data that has just been sent to a
            specific controller.
  input:    - Controller Number (0 to 3) and -1 signalling end of
              processing the pif ram.
            - Pointer of data to be processed.
  output:   none

  note:     This function is only needed if the DLL is allowing raw
            data, or the plugin is set to raw

            the data that is being processed looks like this:
            initilize controller: 01 03 00 FF FF FF
            read controller:      01 04 01 FF FF FF FF
*******************************************************************/
EXPORT void CALL ControllerCommand(int Control, unsigned char *Command)
{
    unsigned char *Data = &Command[5];

    if (Control == -1)
        return;

    switch (Command[2])
    {
        case RD_GETSTATUS:
#ifdef _DEBUG
            DebugMessage(M64MSG_INFO, "Get status");
#endif
            break;
        case RD_READKEYS:
#ifdef _DEBUG
            DebugMessage(M64MSG_INFO, "Read keys");
#endif
            break;
        case RD_READPAK:
#ifdef _DEBUG
            DebugMessage(M64MSG_INFO, "Read pak");
#endif
            if (controller[Control].control->Plugin == PLUGIN_RAW)
            {
                unsigned int dwAddress = (Command[3] << 8) + (Command[4] & 0xE0);

                if(( dwAddress >= 0x8000 ) && ( dwAddress < 0x9000 ) )
                    memset( Data, 0x80, 32 );
                else
                    memset( Data, 0x00, 32 );

                Data[32] = DataCRC( Data, 32 );
            }
            break;
        case RD_WRITEPAK:
#ifdef _DEBUG
            DebugMessage(M64MSG_INFO, "Write pak");
#endif
            if (controller[Control].control->Plugin == PLUGIN_RAW)
            {

printf( "RD_WRITEPAK, and control->Plugin is PLUGIN_RAW!" );

                unsigned int dwAddress = (Command[3] << 8) + (Command[4] & 0xE0);
              if (dwAddress == PAK_IO_RUMBLE && *Data)
                    DebugMessage(M64MSG_VERBOSE, "Triggering rumble pack.");

//#ifdef ANDROID
                if( dwAddress == PAK_IO_RUMBLE )
                {
printf( "dwAddress is PAK_IO_RUMBLE!" );
                    if( *Data )
                    {
printf( "*Data exists! Vibrating..." );
                        DebugMessage( M64MSG_INFO, "Android, activating device vibrator" );
                        // TODO: Implement vibration interface to java
                        // Android_JNI_Vibrate( 1 );
                    }
                    else
                    {
printf( "*Data doesn't exist! Stopping Vibration..." );
                        DebugMessage( M64MSG_INFO, "Android, deactivating device vibrator" );
                        // TODO: Implement vibration interface to java
                        // Android_JNI_Vibrate( 0 );
                    }
                }
else
{
printf( "dwAddress is not PAK_IO_RUMBLE" );
}
//#endif 

#ifdef __linux__
                struct input_event play;
                if( dwAddress == PAK_IO_RUMBLE && controller[Control].event_joystick != 0)
                {
                    if( *Data )
                    {
                        play.type = EV_FF;
                        play.code = ffeffect[Control].id;
                        play.value = 1;

                        if (write(controller[Control].event_joystick, (const void*) &play, sizeof(play)) == -1)
                            perror("Error starting rumble effect");

                    }
                    else
                    {
                        play.type = EV_FF;
                        play.code = ffeffect[Control].id;
                        play.value = 0;

                        if (write(controller[Control].event_joystick, (const void*) &play, sizeof(play)) == -1)
                            perror("Error stopping rumble effect");
                    }
                }
#endif //__linux__
                Data[32] = DataCRC( Data, 32 );
            }

else
{
printf( "RD_WRITEPAK, but control->Plugin not PLUGIN_RAW" );
}

            break;
        case RD_RESETCONTROLLER:
#ifdef _DEBUG
            DebugMessage(M64MSG_INFO, "Reset controller");
#endif
            break;
        case RD_READEEPROM:
#ifdef _DEBUG
            DebugMessage(M64MSG_INFO, "Read eeprom");
#endif
            break;
        case RD_WRITEEPROM:
#ifdef _DEBUG
            DebugMessage(M64MSG_INFO, "Write eeprom");
#endif
            break;
        }
}

/******************************************************************
  Function: GetKeys
  Purpose:  To get the current state of the controllers buttons.
  input:    - Controller Number (0 to 3)
            - A pointer to a BUTTONS structure to be filled with
            the controller state.
  output:   none
*******************************************************************/
EXPORT void CALL GetKeys( int Control, BUTTONS *Keys )
{
    static int mousex_residual = 0;
    static int mousey_residual = 0;
    int b, axis_val;
    unsigned char mstate;

///// paulscode, handle input from the virtual gamepad
    doVirtualGamePad();
////

#ifdef _DEBUG
    DebugMessage(M64MSG_VERBOSE, "Controller #%d value: 0x%8.8X", Control, *(int *)&controller[Control].buttons );
#endif
    *Keys = controller[Control].buttons;

    /* handle mempack / rumblepak switching (only if rumble is active on joystick) */
#ifdef __linux__
    if (controller[Control].event_joystick != 0)
    {
        struct input_event play;
        static unsigned int SwitchPackTime[4] = {0, 0, 0, 0}, SwitchPackType[4] = {0, 0, 0, 0};
        // when the user switches packs, we should mimick the act of removing 1 pack, and then inserting another 1 second later
        if (controller[Control].buttons.Value & button_bits[14])
        {
            //SwitchPackTime[Control] = SDL_GetTicks();         // time at which the 'switch pack' command was given
            // TODO: Implement an alternative to SDL_GetTicks()
            SwitchPackTime[Control] = 0;
            SwitchPackType[Control] = PLUGIN_MEMPAK;          // type of new pack to insert
            controller[Control].control->Plugin = PLUGIN_NONE;// remove old pack
            play.type = EV_FF;
            play.code = ffweak[Control].id;
            play.value = 1;
            if (write(controller[Control].event_joystick, (const void*) &play, sizeof(play)) == -1)
                perror("Error starting rumble effect");
        }
        if (controller[Control].buttons.Value & button_bits[15])
        {
            //SwitchPackTime[Control] = SDL_GetTicks();         // time at which the 'switch pack' command was given
            // TODO: Implement an alternative to SDL_GetTicks()
            SwitchPackTime[Control] = 0;
            SwitchPackType[Control] = PLUGIN_RAW;             // type of new pack to insert
            controller[Control].control->Plugin = PLUGIN_NONE;// remove old pack
            play.type = EV_FF;
            play.code = ffstrong[Control].id;
            play.value = 1;
            if (write(controller[Control].event_joystick, (const void*) &play, sizeof(play)) == -1)
                perror("Error starting rumble effect");
        }
        // handle inserting new pack if the time has arrived
        //int now = SDL_GetTicks();
        // TODO: Implement an alternative to SDL_GetTicks()
        int now = 0;
        if (SwitchPackTime[Control] != 0 && (now - SwitchPackTime[Control]) >= 1000)
        {
            controller[Control].control->Plugin = SwitchPackType[Control];
            SwitchPackTime[Control] = 0;
        }
    }
#endif /* __linux__ */

    controller[Control].buttons.Value = 0;
}

static void InitiateRumble(int cntrl)
{
#ifdef __linux__
    DIR* dp;
    struct dirent* ep;
    unsigned long features[4];
    char temp[128];
    char temp2[128];
    int iFound = 0;

    controller[cntrl].event_joystick = 0;

    sprintf(temp,"/sys/class/input/js%d/device", controller[cntrl].device);
    dp = opendir(temp);

    if(dp==NULL)
        return;

    while ((ep=readdir(dp)))
        {
        if (strncmp(ep->d_name, "event",5)==0)
            {
            sprintf(temp, "/dev/input/%s", ep->d_name);
            iFound = 1;
            break;
            }
        else if(strncmp(ep->d_name,"input:event", 11)==0)
            {
            sscanf(ep->d_name, "input:%s", temp2);
            sprintf(temp, "/dev/input/%s", temp2);
            iFound = 1;
            break;
            }
        else if(strncmp(ep->d_name,"input:input", 11)==0)
            {
            strcat(temp, "/");
            strcat(temp, ep->d_name);
            closedir (dp);
            dp = opendir(temp);
            if(dp==NULL)
                return;
            }
       }

    closedir(dp);

    if (!iFound)
    {
        DebugMessage(M64MSG_WARNING, "Couldn't find input event for rumble support.");
        return;
    }

    controller[cntrl].event_joystick = open(temp, O_RDWR);
    if(controller[cntrl].event_joystick==-1)
        {
        DebugMessage(M64MSG_WARNING, "Couldn't open device file '%s' for rumble support.", temp);
        controller[cntrl].event_joystick = 0;
        return;
        }

    if(ioctl(controller[cntrl].event_joystick, EVIOCGBIT(EV_FF, sizeof(unsigned long) * 4), features)==-1)
        {
        DebugMessage(M64MSG_WARNING, "Linux kernel communication failed for force feedback (rumble).\n");
        controller[cntrl].event_joystick = 0;
        return;
        }

    if(!test_bit(FF_RUMBLE, features))
        {
        DebugMessage(M64MSG_WARNING, "No rumble supported on N64 joystick #%i", cntrl + 1);
        controller[cntrl].event_joystick = 0;
        return;
        }

    ffeffect[cntrl].type = FF_RUMBLE;
    ffeffect[cntrl].id = -1;
    ffeffect[cntrl].u.rumble.strong_magnitude = 0xFFFF;
    ffeffect[cntrl].u.rumble.weak_magnitude = 0xFFFF;

    ioctl(controller[cntrl].event_joystick, EVIOCSFF, &ffeffect[cntrl]);

    ffstrong[cntrl].type = FF_RUMBLE;
    ffstrong[cntrl].id = -1;
    ffstrong[cntrl].u.rumble.strong_magnitude = 0xFFFF;
    ffstrong[cntrl].u.rumble.weak_magnitude = 0x0000;
    ffstrong[cntrl].replay.length = 500;
    ffstrong[cntrl].replay.delay = 0;

    ioctl(controller[cntrl].event_joystick, EVIOCSFF, &ffstrong[cntrl]);

    ffweak[cntrl].type = FF_RUMBLE;
    ffweak[cntrl].id = -1;
    ffweak[cntrl].u.rumble.strong_magnitude = 0x0000;
    ffweak[cntrl].u.rumble.weak_magnitude = 0xFFFF;
    ffweak[cntrl].replay.length = 500;
    ffweak[cntrl].replay.delay = 0;

    ioctl(controller[cntrl].event_joystick, EVIOCSFF, &ffweak[cntrl]);

    DebugMessage(M64MSG_INFO, "Rumble activated on N64 joystick #%i", cntrl + 1);
#endif /* __linux__ */
}

/******************************************************************
  Function: InitiateControllers
  Purpose:  This function initialises how each of the controllers
            should be handled.
  input:    - The handle to the main window.
            - A controller structure that needs to be filled for
              the emulator to know how to handle each controller.
  output:   none
*******************************************************************/
EXPORT void CALL InitiateControllers(CONTROL_INFO ControlInfo)
{
    int i;

    // reset controllers
    memset( controller, 0, sizeof( SController ) * 4 );
    // set our CONTROL struct pointers to the array that was passed in to this function from the core
    // this small struct tells the core whether each controller is plugged in, and what type of pak is connected
    for (i = 0; i < 4; i++)
        controller[i].control = ControlInfo.Controls + i;

    for( i = 0; i < 4; i++ )
    {
        // plug in all controllers
        controller[i].control->Present = 1;

        // test for rumble support for this joystick
        InitiateRumble(i);
        // if rumble not supported, switch to mempack
        if (controller[i].control->Plugin == PLUGIN_RAW && controller[i].event_joystick == 0)
            controller[i].control->Plugin = PLUGIN_MEMPAK;
    }

    DebugMessage(M64MSG_INFO, "%s version %i.%i.%i initialized.", PLUGIN_NAME, VERSION_PRINTF_SPLIT(PLUGIN_VERSION));
}

/******************************************************************
  Function: ReadController
  Purpose:  To process the raw data in the pif ram that is about to
            be read.
  input:    - Controller Number (0 to 3) and -1 signalling end of
              processing the pif ram.
            - Pointer of data to be processed.
  output:   none
  note:     This function is only needed if the DLL is allowing raw
            data.
*******************************************************************/
EXPORT void CALL ReadController(int Control, unsigned char *Command)
{
#ifdef _DEBUG
    if (Command != NULL)
        DebugMessage(M64MSG_INFO, "Raw Read (cont=%d):  %02X %02X %02X %02X %02X %02X", Control,
                     Command[0], Command[1], Command[2], Command[3], Command[4], Command[5]);
#endif
}

/******************************************************************
  Function: RomClosed
  Purpose:  This function is called when a rom is closed.
  input:    none
  output:   none
*******************************************************************/
EXPORT void CALL RomClosed(void)
{
}

/******************************************************************
  Function: RomOpen
  Purpose:  This function is called when a rom is open. (from the
            emulation thread)
  input:    none
  output:   none
*******************************************************************/
EXPORT int CALL RomOpen(void)
{
    return 1;
}

/******************************************************************
  Function: SDL_KeyDown
  Purpose:  To pass the SDL_KeyDown message from the emulator to the
            plugin.
  input:    keymod and keysym of the SDL_KEYDOWN message.
  output:   none
*******************************************************************/
EXPORT void CALL SDL_KeyDown(int keymod, int keysym)
{
}

/******************************************************************
  Function: SDL_KeyUp
  Purpose:  To pass the SDL_KeyUp message from the emulator to the
            plugin.
  input:    keymod and keysym of the SDL_KEYUP message.
  output:   none
*******************************************************************/
EXPORT void CALL SDL_KeyUp(int keymod, int keysym)
{
}

