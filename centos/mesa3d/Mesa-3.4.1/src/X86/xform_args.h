/* $Id: xform_args.h,v 1.1.2.1 2000/10/22 23:10:51 gareth Exp $ */

/*
 * Mesa 3-D graphics library
 * Version:  3.4
 *
 * Copyright (C) 1999-2000  Brian Paul   All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * BRIAN PAUL BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN
 * AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/*
 * Transform function interface for assembly code.  Simply define
 * FRAME_OFFSET to the number of bytes pushed onto the stack before
 * using the ARG_* argument macros.
 *
 * Gareth Hughes <gareth@valinux.com>
 */

#ifndef __XFORM_ARGS_H__
#define __XFORM_ARGS_H__

/* Offsets into GLvector4f
 */
#define V4F_DATA	0
#define V4F_START	4
#define V4F_COUNT	8
#define V4F_STRIDE	12
#define V4F_SIZE	16
#define V4F_FLAGS	20

/* GLvector4f flags
 */
#define VEC_SIZE_1	1
#define VEC_SIZE_2	3
#define VEC_SIZE_3	7
#define VEC_SIZE_4	15

/* Offsets for transform_func arguments
 *
 * typedef void (*transform_func)( GLvector4f *to_vec,
 *				   const GLfloat m[16],
 *				   const GLvector4f *from_vec,
 *				   const GLubyte *clipmask,
 *				   const GLubyte flag );
 */
#define OFFSET_DEST	4
#define OFFSET_MATRIX	8
#define OFFSET_SOURCE	12
#define OFFSET_CLIP	16
#define OFFSET_FLAG	20

#define ARG_DEST	REGOFF(FRAME_OFFSET+OFFSET_DEST, ESP)
#define ARG_MATRIX 	REGOFF(FRAME_OFFSET+OFFSET_MATRIX, ESP)
#define ARG_SOURCE 	REGOFF(FRAME_OFFSET+OFFSET_SOURCE, ESP)
#define ARG_CLIP 	REGOFF(FRAME_OFFSET+OFFSET_CLIP, ESP)
#define ARG_FLAG 	REGOFF(FRAME_OFFSET+OFFSET_FLAG, ESP)

#endif
