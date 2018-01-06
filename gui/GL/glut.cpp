#include "glut.h"

void renderScene(void);

void glutAddMenuEntry (const char *label, int value) {};
void glutAddSubMenu (const char *label, int submenu) {};
void glutAttachMenu (int button) {};
int glutCreateMenu (void (*) (int)) {return 0;};
int glutCreateWindow (const char *title) {return 0;};
void glutDisplayFunc (void (*) (void)) {};
int glutGet (GLenum type) {return 0;};
int glutGetModifiers (void) {return 0;};
void glutIdleFunc (void (*)(void)) {};
void glutInit (int *argcp, char **argv) {};
void glutInitDisplayMode (unsigned int mode) {};
void glutInitWindowPosition (int x, int y) {};
void glutInitWindowSize (int width, int height) {};
void glutKeyboardFunc (void (*) (unsigned char key, int x, int y)) {};
void glutMainLoop (void) { 
	while (1)
		renderScene();
};

void glutMotionFunc (void (*) (int x, int y)) {};
void glutMouseFunc (void (*) (int button, int state, int x, int y)) {};
void glutReshapeFunc (void (*) (int width, int height)) {};
void glutSwapBuffers (void) {};
void gluLookAt (GLdouble eyeX, GLdouble eyeY, GLdouble eyeZ, 
		GLdouble centerX, GLdouble centerY, GLdouble centerZ,
		GLdouble upX, GLdouble upY, GLdouble upZ) {};

void glutBitmapCharacter(void *font, int character) { }
int glutBitmapWidth(void *font, int character) { return 0; }
void glutStrokeCharacter(void *font, int character) { }
int glutStrokeWidth(void *font, int character) { return 0; }
int glutBitmapLength(void *font, const unsigned char *string) { return 0; }
int glutStrokeLength(void *font, const unsigned char *string) { return 0; }

void gluOrtho2D (GLdouble left, GLdouble right, GLdouble bottom, GLdouble top) {}
void glMultMatrixf(const GLfloat * m ){}
void glutWireSphere(GLdouble radius, GLint slices, GLint stacks){}
