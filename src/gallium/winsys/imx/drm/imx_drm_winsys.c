/*
 * Copyright (C) 2016 Christian Gmeiner <christian.gmeiner@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Authors:
 *    Christian Gmeiner <christian.gmeiner@gmail.com>
 */

#include "imx_drm_public.h"
#include "etnaviv/drm/etnaviv_drm_public.h"
#include "freedreno/drm/freedreno_drm_public.h"
#include "loader/loader.h"
#include "renderonly/renderonly.h"

#include <fcntl.h>
#include <unistd.h>

struct pipe_screen *imx_drm_screen_create(int fd)
{
   struct renderonly ro = {
      .create_for_resource = renderonly_create_kms_dumb_buffer_for_resource,
      .kms_fd = fd,
   };
   struct pipe_screen *screen;

   printf("imx_drm_screen_create\n");

#if defined(GALLIUM_ETNAVIV)
   ro.gpu_fd = loader_open_render_node("etnaviv");
   printf("a %i\n", ro.gpu_fd);
   if (ro.gpu_fd >= 0) {
      screen = etna_drm_screen_create_renderonly(&ro);
      if (screen)
	     return screen;
      close(ro.gpu_fd);
   }
#endif

#if defined(GALLIUM_FREEDRENO)
   //ro.gpu_fd = loader_open_render_node("msm");
   ro.gpu_fd = open("/dev/dri/renderD128", O_RDWR);
   //loader_open_render_node("msm");
   if (ro.gpu_fd >= 0) {
      screen = fd_drm_screen_create_renderonly(&ro);
      if (screen)
	     return screen;
      close(ro.gpu_fd);
   }
#endif

   return NULL;
}
