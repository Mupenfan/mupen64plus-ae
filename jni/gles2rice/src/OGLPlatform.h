/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *   Mupen64plus - OGLPlatform.h                                           *
 *   Mupen64Plus homepage: http://code.google.com/p/mupen64plus/           *
 *   Copyright (C) 2013 Paul Lamb                                          *
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

#ifndef _OGL_PLATFORM_H_
#define _OGL_PLATFORM_H_

#ifdef M64P_USE_GLES2
#include "GLES2/gl2.h"
#include "GLES2/gl2ext.h"
#define GLSL_VERSION "100"

// Constants
#define GL_ADD_SIGNED_ARB                   GL_ADD_SIGNED
#define GL_CLAMP                            GL_CLAMP_TO_EDGE
#define GL_COMBINE_RGB_ARB                  GL_COMBINE_RGB
#define GL_CONSTANT_ARB                     GL_CONSTANT
#define GL_INTERPOLATE_ARB                  GL_INTERPOLATE
#define GL_MAX_TEXTURE_UNITS_ARB            GL_MAX_TEXTURE_UNITS
#define GL_MIRRORED_REPEAT_ARB              GL_MIRRORED_REPEAT
#define GL_OPERAND0_RGB_ARB                 GL_OPERAND0_RGB
#define GL_OPERAND1_RGB_ARB                 GL_OPERAND1_RGB
#define GL_OPERAND2_RGB_ARB                 GL_OPERAND2_RGB
#define GL_OPERAND0_RGB_ALPHA_ARB           GL_OPERAND0_ALPHA
#define GL_OPERAND1_RGB_ALPHA_ARB           GL_OPERAND1_ALPHA
#define GL_OPERAND2_RGB_ALPHA_ARB           GL_OPERAND2_ALPHA
#define GL_PREVIOUS_ARB                     GL_PREVIOUS
#define GL_PRIMARY_COLR_ARB                 GL_PRIMARY_COLOR
#define GL_SOURCE0_RGB_ARB                  GL_SRC0_RGB
#define GL_SOURCE1_RGB_ARB                  GL_SRC1_RGB
#define GL_SOURCE2_RGB_ARB                  GL_SRC2_RGB
#define GL_SOURCE0_ALPHA_ARB                GL_SRC0_ALPHA
#define GL_SOURCE1_ALPHA_ARB                GL_SRC1_ALPHA
#define GL_SOURCE2_ALPHA_ARB                GL_SRC2_ALPHA
#define GL_SUBTRACT_ARB                     GL_SUBTRACT
#define GL_TEXTURE0_ARB                     GL_TEXTURE0
#define GL_TEXTURE1_ARB                     GL_TEXTURE1
#define GL_TEXTURE2_ARB                     GL_TEXTURE2
#define GL_TEXTURE3_ARB                     GL_TEXTURE3
#define GL_TEXTURE4_ARB                     GL_TEXTURE4
#define GL_TEXTURE5_ARB                     GL_TEXTURE5
#define GL_TEXTURE6_ARB                     GL_TEXTURE6
#define GL_TEXTURE7_ARB                     GL_TEXTURE7
//#define GL_BGRA                            GL_BGRA_IMG

// Strings
#define M64P_GL_ARB_FRAGMENT_PROGRAM        "GL_fragment_program"
#define M64P_GL_ARB_FRAGMENT_SHADER         "GL_fragment_shader"
#define M64P_GL_ARB_MULTITEXTURE            "GL_multitexture"
#define M64P_GL_ARB_TEXTURE_BORDER_CLAMP    "GL_texture_border_clamp"
#define M64P_GL_ARB_TEXTURE_ENV_ADD         "GL_texture_env_add"
#define M64P_GL_ARB_TEXTURE_ENV_COMBINE     "GL_texture_env_combine"
#define M64P_GL_ARB_TEXTURE_ENV_CROSSBAR    "GL_texture_env_crossbar"

// Functions
#define pglActiveTexture                    glActiveTexture
#define pglActiveTextureARB                 glActiveTexture
#define glTexEnvi(x,y,z)                    // Unavailable in GLES2
#define glTexEnvfv(x,y,z)                   // Unavailable in GLES2
// From http://pandorawiki.org/Porting_to_GLES_from_GL
#define GLdouble                            GLfloat
#define glClearDepth                        glClearDepthf
#define glColor4fv(a)                       glColor4f(a[0], a[1], a[2], a[3])
#define glColor3fv(a)                       glColor4f(a[0], a[1], a[2], 1.0f)
#define glColor3f(a,b,c)                    glColor4f(a, b, c, 1.0f)
#define glFrustum                           glFrustumf
#define glOrtho                             glOrthof

#else
#include <SDL_opengl.h>
#include <GL\glew.h>
#include "OGLCombinerNV.h"
#include "OGLCombinerTNT2.h"
#include "OGLExtensions.h"
#define GLSL_VERSION "120"

// Strings
#define M64P_GL_ARB_FRAGMENT_PROGRAM        "GL_ARB_fragment_program"
#define M64P_GL_ARB_FRAGMENT_SHADER         "GL_ARB_fragment_shader"
#define M64P_GL_ARB_MULTITEXTURE            "GL_ARB_multitexture"
#define M64P_GL_ARB_TEXTURE_BORDER_CLAMP    "GL_ARB_texture_border_clamp"
#define M64P_GL_ARB_TEXTURE_ENV_ADD         "GL_ARB_texture_env_add"
#define M64P_GL_ARB_TEXTURE_ENV_COMBINE     "GL_ARB_texture_env_combine"
#define M64P_GL_ARB_TEXTURE_ENV_CROSSBAR    "GL_ARB_texture_env_crossbar"
#endif

#endif // _OGL_PLATFORM_H_
