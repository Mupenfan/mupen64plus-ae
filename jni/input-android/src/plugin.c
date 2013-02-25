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

#include <string.h>
#include <jni.h>
#include <android/log.h>

#include "plugin.h"
#include "version.h"

// Internal function declarations
static unsigned char DataCRC( unsigned char*, int );

// Global variable definitions
SController controller[4];

// Internal variable definitions
static void (*_DebugCallback)( void *, int, const char * ) = NULL;
static void *_debugCallContext = NULL;
static int _pluginInitialized = 0;
static CONTROL _core_controlinfo_storage[4];
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

void DefaultDebugCallback( void *context, int level, const char *message )
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
        _DebugCallback = DefaultDebugCallback;

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
        *capabilities = 0;

    return M64ERR_SUCCESS;
}

EXPORT m64p_error CALL PluginStartup( m64p_dynlib_handle coreLibHandle, void *context, void (*DebugCallback)( void *, int, const char * ) )
{
    DebugMessage( M64MSG_INFO, "PluginStartup" );

    if( _pluginInitialized )
        return M64ERR_ALREADY_INIT;

    _DebugCallback = DebugCallback;
    _debugCallContext = context;

    // Reset the controller data structure
    memset(controller, 0, sizeof(SController) * 4);

    // Define the storage location for controller data
    int i;
    for( i = 0; i < 4; i++ )
        controller[i].control = _core_controlinfo_storage + i;

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

EXPORT void CALL InitiateControllers( CONTROL_INFO controlInfo )
{
    DebugMessage( M64MSG_INFO, "InitiateControllers" );

    // Reset controllers
    memset( controller, 0, sizeof(SController) * 4 );

    int i;
    for( i = 0; i < 4; i++ )
    {
        // Record pointers to the controller data
        controller[i].control = controlInfo.Controls + i;

        // Plug in all controllers
        // TODO: Obtain this from JNI
        controller[i].control->Present = 1;
    }
}

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

    // Assign the output data
    *keys = controller[controllerNum].buttons;
    controller[controllerNum].buttons.Value = 0;
}

EXPORT void CALL ControllerCommand( int Control, unsigned char *Command )
{
    DebugMessage( M64MSG_INFO, "ControllerCommand" );
}

EXPORT void CALL ReadController( int Control, unsigned char *Command )
{
    DebugMessage( M64MSG_INFO, "ReadController" );
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
