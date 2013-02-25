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

#include <string.h> // memset, NULL
#include <jni.h>
#include <android/log.h>

#include "plugin.h"
#include "version.h"

// External function declarations
extern void Android_JNI_Vibrate( int active );

// Internal function declarations
static unsigned char DataCRC( unsigned char*, int );

// Global variable definitions
SController controller[4];

// Internal variable definitions
static void (*_DebugCallback)( void *, int, const char * ) = NULL;
static void *_debugCallContext = NULL;
static int _pluginInitialized = 0;
static CONTROL _temp_core_controlinfo[4];
unsigned char _androidButtonState[4][16];
signed char _androidAnalogState[4][2];

// Internal constant definitions
static const unsigned short const BUTTON_BITS[] =
{
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

//*****************************************************************************
// Mupen64Plus debug function definitions
//*****************************************************************************

void DebugCallback(void *context, int level, const char *message)
{
    switch( level )
    {
    case M64MSG_ERROR:
        __android_log_print( ANDROID_LOG_ERROR, "input-android", message );
        break;
    case M64MSG_WARNING:
        __android_log_print( ANDROID_LOG_WARN, "input-android", message );
        break;
    case M64MSG_INFO:
        __android_log_print( ANDROID_LOG_INFO, "input-android", message );
        break;
    case M64MSG_STATUS:
        __android_log_print( ANDROID_LOG_DEBUG, "input-android", message );
        break;
    case M64MSG_VERBOSE:
    default:
        //__android_log_print( ANDROID_LOG_VERBOSE, "input-android", message );
        break;
    }
}

void DebugMessage( int level, const char *message, ... )
{
    char msgbuf[1024];
    va_list args;

    if( _DebugCallback == NULL )
        _DebugCallback = DebugCallback;

    va_start( args, message );
    vsprintf( msgbuf, message, args );

    ( *_DebugCallback )( _debugCallContext, level, msgbuf );

    va_end( args );
}

//*****************************************************************************
// Mupen64Plus common plugin function definitions
//*****************************************************************************

EXPORT m64p_error CALL PluginGetVersion( m64p_plugin_type *pluginType, int *pluginVersion, int *apiVersion, const char **pluginNamePtr, int *capabilities )
{
    DebugMessage( M64MSG_INFO, "PluginGetVersion" );

    if( pluginType != NULL )
        *pluginType = M64PLUGIN_INPUT;

    if( pluginVersion != NULL )
        *pluginVersion = PLUGIN_VERSION;

    if( apiVersion != NULL )
        *apiVersion = INPUT_PLUGIN_API_VERSION;

    if( pluginNamePtr != NULL )
        *pluginNamePtr = PLUGIN_NAME;

    if( capabilities != NULL )
    {
        *capabilities = 0;
    }

    return M64ERR_SUCCESS;
}

EXPORT m64p_error CALL PluginStartup( m64p_dynlib_handle coreLibHandle, void *context, void (*DebugCallback)( void *, int, const char * ) )
{
    DebugMessage( M64MSG_INFO, "PluginStartup" );

    if( _pluginInitialized )
        return M64ERR_ALREADY_INIT;

    _DebugCallback = DebugCallback;
    _debugCallContext = context;

    memset(controller, 0, sizeof(SController) * 4);

    /* set CONTROL struct pointers to the temporary static array */
    /* this small struct is used to tell the core whether each controller is plugged in, and what type of pak is connected */
    /* we only need it so that we can call load_configuration below, to auto-config for a GUI front-end */
    int i;
    for (i = 0; i < 4; i++)
        controller[i].control = _temp_core_controlinfo + i;

    _pluginInitialized = 1;
    return M64ERR_SUCCESS;
}

EXPORT m64p_error CALL PluginShutdown()
{
    DebugMessage( M64MSG_INFO, "PluginShutdown" );

    if( !_pluginInitialized )
        return M64ERR_NOT_INIT;

    _DebugCallback = NULL;
    _debugCallContext = NULL;
    _pluginInitialized = 0;
    return M64ERR_SUCCESS;
}

//*****************************************************************************
// Mupen64Plus input plugin function definitions
//*****************************************************************************

/******************************************************************
  Function: InitiateControllers
  Purpose:  This function initializes how each of the controllers
            should be handled.
  input:    - The handle to the main window.
            - A controller structure that needs to be filled for
              the emulator to know how to handle each controller.
  output:   none
*******************************************************************/

EXPORT void CALL InitiateControllers( CONTROL_INFO controlInfo )
{
    DebugMessage( M64MSG_INFO, "InitiateControllers" );

    // Reset controllers
    memset( controller, 0, sizeof(SController) * 4 );

    // set our CONTROL struct pointers to the array that was passed in to this function from the core
    // this small struct tells the core whether each controller is plugged in, and what type of pak is connected
    int i;
    for( i = 0; i < 4; i++ )
        controller[i].control = controlInfo.Controls + i;

    for( i = 0; i < 4; i++ )
    {
        // Plug in all controllers
        // TODO: Obtain this from JNI
        controller[i].control->Present = 1;

        // Test for rumble support for this joystick
        // If rumble not supported, switch to mempack
        // TODO: Re-implement if needed
    }

    DebugMessage( M64MSG_INFO, "%s version %i.%i.%i initialized.", PLUGIN_NAME, VERSION_PRINTF_SPLIT(PLUGIN_VERSION) );
}

/******************************************************************
  Function: GetKeys
  Purpose:  To get the current state of the controllers buttons.
  input:    - Controller Number (0 to 3)
            - A pointer to a BUTTONS structure to be filled with
            the controller state.
  output:   none
*******************************************************************/
EXPORT void CALL GetKeys( int controllerNum, BUTTONS *keys )
{
    int b, c;
    for( c = 0; c < 4; c++ )
    {
        // Set the button bits
        for( b = 0; b < 16; b++ )
        {
            if( _androidButtonState[c][b] )
                controller[c].buttons.Value |= BUTTON_BITS[b];
        }

        // Set the analog bytes
        if( _androidAnalogState[c][0] || _androidAnalogState[c][1] )
        {
            // only report the stick position if it isn't back at the center
            controller[c].buttons.X_AXIS = _androidAnalogState[c][0];
            controller[c].buttons.Y_AXIS = _androidAnalogState[c][1];
        }
    }

    DebugMessage( M64MSG_VERBOSE, "Controller #%d value: 0x%8.8X", controllerNum, *(int *)&controller[controllerNum].buttons );

    // Assign the output data
    *keys = controller[controllerNum].buttons;
    controller[controllerNum].buttons.Value = 0;
}

EXPORT void CALL ControllerCommand( int Control, unsigned char *Command )
{
    DebugMessage( M64MSG_INFO, "ControllerCommand" );
    unsigned char *Data = &Command[5];

    if( Control == -1 )
        return;

    switch( Command[2] )
    {
    case RD_GETSTATUS:
        DebugMessage(M64MSG_VERBOSE, "Get status");
        break;
    case RD_READKEYS:
        DebugMessage(M64MSG_VERBOSE, "Read keys");
        break;
    case RD_READPAK:
        DebugMessage(M64MSG_VERBOSE, "Read pak");
        if( controller[Control].control->Plugin == PLUGIN_RAW )
        {
            unsigned int dwAddress = ( Command[3] << 8 ) + ( Command[4] & 0xE0 );

            if( ( dwAddress >= 0x8000 ) && ( dwAddress < 0x9000 ) )
                memset( Data, 0x80, 32 );
            else
                memset( Data, 0x00, 32 );

            Data[32] = DataCRC( Data, 32 );
        }
        break;
    case RD_WRITEPAK:
        DebugMessage(M64MSG_VERBOSE, "Write pak");
        if( controller[Control].control->Plugin == PLUGIN_RAW )
        {
            DebugMessage( M64MSG_VERBOSE, "RD_WRITEPAK, and control->Plugin is PLUGIN_RAW!" );

            unsigned int dwAddress = ( Command[3] << 8 ) + ( Command[4] & 0xE0 );
            if( dwAddress == PAK_IO_RUMBLE && *Data )
                DebugMessage( M64MSG_VERBOSE, "Triggering rumble pack." );

//#ifdef ANDROID
            if( dwAddress == PAK_IO_RUMBLE )
            {
                DebugMessage( M64MSG_VERBOSE, "dwAddress is PAK_IO_RUMBLE!" );
                if( *Data )
                {
                    DebugMessage( M64MSG_VERBOSE, "*Data exists! Vibrating..." );
                    DebugMessage( M64MSG_INFO, "Android, activating device vibrator" );
                    // TODO: Implement vibration interface to java
                    // Android_JNI_Vibrate( 1 );
                }
                else
                {
                    DebugMessage( M64MSG_VERBOSE, "*Data doesn't exist! Stopping Vibration..." );
                    DebugMessage( M64MSG_INFO, "Android, deactivating device vibrator" );
                    // TODO: Implement vibration interface to java
                    // Android_JNI_Vibrate( 0 );
                }
            }
            else
            {
                DebugMessage( M64MSG_VERBOSE, "dwAddress is not PAK_IO_RUMBLE" );
            }
//#endif 

            Data[32] = DataCRC( Data, 32 );
        }

        else
        {
            DebugMessage( M64MSG_VERBOSE, "RD_WRITEPAK, but control->Plugin not PLUGIN_RAW" );
        }

        break;
    case RD_RESETCONTROLLER:
        DebugMessage(M64MSG_VERBOSE, "Reset controller");
        break;
    case RD_READEEPROM:
        DebugMessage(M64MSG_VERBOSE, "Read eeprom");
        break;
    case RD_WRITEEPROM:
        DebugMessage(M64MSG_VERBOSE, "Write eeprom");
        break;
    }
}

EXPORT void CALL ReadController( int Control, unsigned char *Command )
{
}

EXPORT void CALL RomClosed()
{
}

EXPORT int CALL RomOpen()
{
    return 1;
}

EXPORT void CALL SDL_KeyDown( int keymod, int keysym )
{
}

EXPORT void CALL SDL_KeyUp( int keymod, int keysym )
{
}

//*****************************************************************************
// Internal function definitions
//*****************************************************************************

static unsigned char DataCRC( unsigned char *data, int length )
{
    unsigned char remainder = data[0];

    int byteCount = 1;
    unsigned char bBit = 0;

    while( byteCount <= length )
    {
        int highBit = ( ( remainder & 0x80 ) != 0 );
        remainder = remainder << 1;

        remainder += ( byteCount < length && data[byteCount] & ( 0x80 >> bBit ) ) ? 1 : 0;

        remainder ^= ( highBit ) ? 0x85 : 0;

        bBit++;
        byteCount += bBit / 8;
        bBit %= 8;
    }

    return remainder;
}

JNIEXPORT void JNICALL Java_paulscode_android_mupen64plusae_CoreInterfaceNative_updateVirtualGamePadStates(
        JNIEnv* env, jclass jcls, jint controllerNum, jbooleanArray mp64pButtons, jint mp64pXAxis, jint mp64pYAxis )
{
    jboolean *elements = (*env)->GetBooleanArrayElements( env, mp64pButtons, NULL );
    int b;
    for( b = 0; b < 16; b++ )
    {
        _androidButtonState[controllerNum][b] = elements[b];
    }
    (*env)->ReleaseBooleanArrayElements( env, mp64pButtons, elements, 0 );

    _androidAnalogState[controllerNum][0] = (signed char) ((int) mp64pXAxis);
    _androidAnalogState[controllerNum][1] = (signed char) ((int) mp64pYAxis);
}
