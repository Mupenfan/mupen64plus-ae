/**
 * Mupen64PlusAE, an N64 emulator for the Android platform
 *
 * Copyright (C) 2013 Paul Lamb
 *
 * This file is part of Mupen64PlusAE.
 *
 * Mupen64PlusAE is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * Mupen64PlusAE is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with Mupen64PlusAE. If
 * not, see <http://www.gnu.org/licenses/>.
 *
 * Authors: littleguy77, Paul Lamb
 */

#include <string.h>
#include <jni.h>
#include <android/log.h>

#include "m64p_plugin.h"

// Internal macros
#define PLUGIN_NAME                 "Mupen64Plus Android Input Plugin"
#define PLUGIN_VERSION              0x010000
#define INPUT_PLUGIN_API_VERSION    0x020000

// Internal variables
static int _pluginInitialized = 0;
static unsigned char _androidButtonState[4][16];
static signed char _androidAnalogState[4][2];

// Internal constants
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

void DebugMessage( int level, const char *message, ... )
{
    char msgbuf[1024];
    va_list args;
    va_start( args, message );
    vsprintf( msgbuf, message, args );
    va_end( args );

    switch( level )
    {
    case M64MSG_ERROR:
        __android_log_print( ANDROID_LOG_ERROR, "input-android", msgbuf );
        break;
    case M64MSG_WARNING:
        __android_log_print( ANDROID_LOG_WARN, "input-android", msgbuf );
        break;
    case M64MSG_INFO:
        __android_log_print( ANDROID_LOG_INFO, "input-android", msgbuf );
        break;
    case M64MSG_STATUS:
        __android_log_print( ANDROID_LOG_DEBUG, "input-android", msgbuf );
        break;
    case M64MSG_VERBOSE:
    default:
        //__android_log_print( ANDROID_LOG_VERBOSE, "input-android", msgbuf );
        break;
    }
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

    _pluginInitialized = 1;
    return M64ERR_SUCCESS;
}

EXPORT m64p_error CALL PluginShutdown()
{
    DebugMessage( M64MSG_INFO, "PluginShutdown" );

    if( !_pluginInitialized )
        return M64ERR_NOT_INIT;

    _pluginInitialized = 0;
    return M64ERR_SUCCESS;
}

//*****************************************************************************
// Mupen64Plus input plugin function definitions
//*****************************************************************************

EXPORT void CALL InitiateControllers( CONTROL_INFO controlInfo )
{
    DebugMessage( M64MSG_INFO, "InitiateControllers" );

    int i;
    for( i = 0; i < 4; i++ )
    {
        // Configure each controller
        // TODO: Obtain this through JNI
        controlInfo.Controls->Present = 1;
        controlInfo.Controls->Plugin = 2;
        controlInfo.Controls->RawData = 0;
    }
}

EXPORT void CALL GetKeys( int controllerNum, BUTTONS *keys )
{
    // Reset the controller state
    keys->Value = 0;

    // Set the button bits
    int b;
    for( b = 0; b < 16; b++ )
    {
        if( _androidButtonState[controllerNum][b] )
            keys->Value |= BUTTON_BITS[b];
    }

    // Set the analog bytes
    if( _androidAnalogState[controllerNum][0] || _androidAnalogState[controllerNum][1] )
    {
        // Only report non-zero analog states
        keys->X_AXIS = _androidAnalogState[controllerNum][0];
        keys->Y_AXIS = _androidAnalogState[controllerNum][1];
    }
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
// JNI exported function definitions
//*****************************************************************************

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
