/* $Id: osdemo.c,v 1.4 2000/03/28 16:59:39 rjfrank Exp $ */

/*
 * Demo of off-screen Mesa rendering
 *
 * See Mesa/include/GL/osmesa.h for documentation of the OSMesa functions.
 *
 * If you want to render BIG images you'll probably have to increase
 * MAX_WIDTH and MAX_HEIGHT in src/config.h.
 *
 * This program is in the public domain.
 *
 * Brian Paul
 *
 * PPM output provided by Joerg Schmalzl.
 * ASCII PPM output added by Brian Paul.
 */


#include <stdio.h>
#include <stdlib.h>
#include "GL/osmesa.h"
#include "GL/glut.h"


#define SAVE_TARGA


#define WIDTH 400
#define HEIGHT 400



static void render_image( void )
{
   GLfloat light_ambient[] = { 0.0, 0.0, 0.0, 1.0 };
   GLfloat light_diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
   GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };
   GLfloat light_position[] = { 1.0, 1.0, 1.0, 0.0 };
   GLfloat red_mat[]   = { 1.0, 0.2, 0.2, 1.0 };
   GLfloat green_mat[] = { 0.2, 1.0, 0.2, 1.0 };
   GLfloat blue_mat[]  = { 0.2, 0.2, 1.0, 1.0 };


   glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
   glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
   glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
   glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    
   glEnable(GL_LIGHTING);
   glEnable(GL_LIGHT0);
   glEnable(GL_DEPTH_TEST);

   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   glOrtho(-2.5, 2.5, -2.5, 2.5, -10.0, 10.0);
   glMatrixMode(GL_MODELVIEW);

   glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

   glPushMatrix();
   glRotatef(20.0, 1.0, 0.0, 0.0);

   glPushMatrix();
   glTranslatef(-0.75, 0.5, 0.0); 
   glRotatef(90.0, 1.0, 0.0, 0.0);
   glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, red_mat );
   glutSolidTorus(0.275, 0.85, 20, 20);
   glPopMatrix();

   glPushMatrix();
   glTranslatef(-0.75, -0.5, 0.0); 
   glRotatef(270.0, 1.0, 0.0, 0.0);
   glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, green_mat );
   glutSolidCone(1.0, 2.0, 16, 1);
   glPopMatrix();

#ifdef GL_HP_occlusion_test
   {
      GLboolean bRet;
      glDepthMask(GL_FALSE);
      glColorMask(GL_FALSE,GL_FALSE,GL_FALSE,GL_FALSE);
      glEnable(GL_OCCLUSION_TEST_HP);
      glGetBooleanv(GL_OCCLUSION_TEST_RESULT_HP,&bRet);

      glPushMatrix();
      glTranslatef(0.75, 0.0, -1.0);
      glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, blue_mat );
      glutSolidSphere(1.0, 20, 20);
      glPopMatrix();

      glGetBooleanv(GL_OCCLUSION_TEST_RESULT_HP,&bRet);
      printf("Occlusion test 1 (result should be 1): %d\n",bRet);

      glDepthMask(GL_TRUE);
      glColorMask(GL_TRUE,GL_TRUE,GL_TRUE,GL_TRUE);
      glDisable(GL_OCCLUSION_TEST_HP);
   }
#endif

   glPushMatrix();
   glTranslatef(0.75, 0.0, -1.0); 
   glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, blue_mat );
   glutSolidSphere(1.0, 20, 20);
   glPopMatrix();

#ifdef GL_HP_occlusion_test
   {
      GLboolean bRet;

      glDepthMask(GL_FALSE);
      glColorMask(GL_FALSE,GL_FALSE,GL_FALSE,GL_FALSE);
      glEnable(GL_OCCLUSION_TEST_HP);
      glGetBooleanv(GL_OCCLUSION_TEST_RESULT_HP,&bRet);

      /* draw a sphere inside the previous sphere */
      glPushMatrix();
      glTranslatef(0.75, 0.0, -1.0);
      glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, blue_mat );
      glutSolidSphere(0.5, 20, 20);
      glPopMatrix();

      glGetBooleanv(GL_OCCLUSION_TEST_RESULT_HP,&bRet);
      printf("Occlusion test 2 (result should be 0): %d\n",bRet);

      glDepthMask(GL_TRUE);
      glColorMask(GL_TRUE,GL_TRUE,GL_TRUE,GL_TRUE);
      glDisable(GL_OCCLUSION_TEST_HP);
   }
#endif

   glPopMatrix();
}



static void
write_targa(const char *filename, const GLubyte *buffer, int width, int height)
{
   FILE *f = fopen( filename, "w" );
   if (f) {
      int i, x, y;
      const GLubyte *ptr = buffer;
      printf ("osdemo, writing tga file \n");
      fputc (0x00, f);	/* ID Length, 0 => No ID	*/
      fputc (0x00, f);	/* Color Map Type, 0 => No color map included	*/
      fputc (0x02, f);	/* Image Type, 2 => Uncompressed, True-color Image */
      fputc (0x00, f);	/* Next five bytes are about the color map entries */
      fputc (0x00, f);	/* 2 bytes Index, 2 bytes length, 1 byte size */
      fputc (0x00, f);
      fputc (0x00, f);
      fputc (0x00, f);
      fputc (0x00, f);	/* X-origin of Image	*/
      fputc (0x00, f);
      fputc (0x00, f);	/* Y-origin of Image	*/
      fputc (0x00, f);
      fputc (WIDTH & 0xff, f);      /* Image Width	*/
      fputc ((WIDTH>>8) & 0xff, f);
      fputc (HEIGHT & 0xff, f);     /* Image Height	*/
      fputc ((HEIGHT>>8) & 0xff, f);
      fputc (0x18, f);		/* Pixel Depth, 0x18 => 24 Bits	*/
      fputc (0x20, f);		/* Image Descriptor	*/
      fclose(f);
      f = fopen( filename, "ab" );  /* reopen in binary append mode */
      for (y=height-1; y>=0; y--) {
         for (x=0; x<width; x++) {
            i = (y*width + x) * 4;
            fputc(ptr[i+2], f); /* write blue */
            fputc(ptr[i+1], f); /* write green */
            fputc(ptr[i], f);   /* write red */
         }
      }
   }
}


static void
write_ppm(const char *filename, const GLubyte *buffer, int width, int height)
{
   const int binary = 0;
   FILE *f = fopen( filename, "w" );
   if (f) {
      int i, x, y;
      const GLubyte *ptr = buffer;
      if (binary) {
         fprintf(f,"P6\n");
         fprintf(f,"# ppm-file created by osdemo.c\n");
         fprintf(f,"%i %i\n", width,height);
         fprintf(f,"255\n");
         fclose(f);
         f = fopen( filename, "ab" );  /* reopen in binary append mode */
         for (y=height-1; y>=0; y--) {
            for (x=0; x<width; x++) {
               i = (y*width + x) * 4;
               fputc(ptr[i], f);   /* write red */
               fputc(ptr[i+1], f); /* write green */
               fputc(ptr[i+2], f); /* write blue */
            }
         }
      }
      else {
         /*ASCII*/
         int counter = 0;
         fprintf(f,"P3\n");
         fprintf(f,"# ascii ppm file created by osdemo.c\n");
         fprintf(f,"%i %i\n", width, height);
         fprintf(f,"255\n");
         for (y=height-1; y>=0; y--) {
            for (x=0; x<width; x++) {
               i = (y*width + x) * 4;
               fprintf(f, " %3d %3d %3d", ptr[i], ptr[i+1], ptr[i+2]);
               counter++;
               if (counter % 5 == 0)
                  fprintf(f, "\n");
            }
         }
      }
      fclose(f);
   }
}



int main( int argc, char *argv[] )
{
   /* Create an RGBA-mode context */
   OSMesaContext ctx = OSMesaCreateContext( GL_RGBA, NULL );

   /* Allocate the image buffer */
   void *buffer = malloc( WIDTH * HEIGHT * 4 );

   /* Bind the buffer to the context and make it current */
   OSMesaMakeCurrent( ctx, buffer, GL_UNSIGNED_BYTE, WIDTH, HEIGHT );

   render_image();

   if (argc>1) {
#ifdef SAVE_TARGA
      write_targa(argv[1], buffer, WIDTH, HEIGHT);
#else
      write_ppm(argv[1], buffer, WIDTH, HEIGHT);
#endif
   }
   else {
      printf("Specify a filename if you want to make an image file\n");
   }

   printf("all done\n");

   /* free the image buffer */
   free( buffer );

   /* destroy the context */
   OSMesaDestroyContext( ctx );

   return 0;
}
