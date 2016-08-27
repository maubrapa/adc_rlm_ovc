#pragma once

//#include <windows.h>
//#include <GL\GL.h>
//#include "GL/gl.h"

struct Vertex{
	float X, Y, Z;
};

struct Vertex2D{
	float X, Y;
};

struct Color3i{
	int r, g, b;
};
struct Color3ub{
	Color3ub(unsigned char _r = 0, unsigned char _g = 0, unsigned char _b = 0){
		r = _r;
		g = _g;
		b = _b;		
	}
	unsigned char r, g, b;
};

struct ImageCoord{
	ImageCoord(int U, int V){
		this->U = U;
		this->V = V;
	}
	ImageCoord(void){
		U=0;
		V=0;
	}
	int U, V;
};

struct Box2D{
	Box2D(void){
		height = 1.0f;
		width = 1.0f;
		pos.X = 0.0f;
		pos.Y = 0.0f;
	}
	Box2D(float _width, float _height, float posX, float posY){
		width = _width;
		height = _height;
		pos.X = posX;
		pos.Y = posY;
	}
	float getArea(void){
		return height*width;
	}
	float height, width;
	Vertex2D pos;
};

inline float clampCharToFloat(unsigned char value){
	return ((float)value)/255.0f;
};
/*inline unsigned char unsign(char value){
	int retVal = value;
	if(value<0) retVal += 128;
	
	return retVal;
};*/
inline unsigned char unsign(char value){
	return (unsigned char)value;
};
inline char sign(unsigned char value){
	return (char)value;
};
inline float module(float val){
	if(val<0.0f) return -val;

	return val;
};
inline int module(int val){
	if(val<0) return -val;

	return val;
};
// Essa função serve só para posicionar o cursor no console do windows.
//inline void placeConsoleCursorAt(int coluna, int linha)
//{
//	static HANDLE console_output_handle = GetStdHandle(STD_OUTPUT_HANDLE);
//	static COORD posicao_no_console;
//
//	posicao_no_console.X = coluna;
//	posicao_no_console.Y = linha;
//	SetConsoleCursorPosition( console_output_handle, posicao_no_console);
//};

//inline void desenhaCruz( int posX = 0, int posY = 0, int tam = 16 ){
//	tam /= 2;
//	
//	ImageCoord	cima(posX, posY-tam),
//				baixo(posX, posY+tam),
//				esq(posX-tam, posY),
//				dir(posX+tam, posY);
//
//	glBegin( GL_LINES );
//	{
//		glVertex2i( cima.U, cima.V );
//		glVertex2i( baixo.U, baixo.V );
//
//		glVertex2i( esq.U, esq.V );
//		glVertex2i( dir.U, dir.V );
//	}
//	glEnd( );
//};

// Essa função desenha uma grade num plano XZ em OpenGL. Bem intuitiva.
//inline void desenhaGradeXZ( float larg, float prof, float x_origem, float z_origem, float step ){
//	float	x_min = -(larg/2.f),
//			z_min = -(prof/2.f);
//	float	x_max = -x_min,
//			z_max = -z_min;
//
//	glBegin( GL_LINES );
//	{
//		for( float z=z_min; z<z_max; z+=step ){
//			for( float x=x_min; x<x_max; x+=step ){
//				// Linhas horizontais
//				glVertex3f( x_min, 0.f, z );
//				glVertex3f( x_max, 0.f, z );
//				// Linhas verticais
//				glVertex3f( x, 0.f, z_min );
//				glVertex3f( x, 0.f, z_max );
//			}
//		}
//	}
//	glEnd();
//	
//	return;
//};

//void drawCircle( float pos_x = 0.0f, float pos_y = 0.0f, float radius = 1.0f, int sides = 12, GLenum mode = GL_POLYGON ){
//	float tetha = 2*PI / sides;
////	float L = radius*(2*cos(PI/sides));
//
//	float v[3];
//	v[2] = 0.f;
//	float t;
//	glBegin( mode );
//		for (int i=0; i<sides; i++){
//			t = (float)i * tetha;
//			v[0] = pos_x + radius*cos(t);
//			v[1] = pos_y + radius*sin(t);
//			glVertex3fv(v);
//		}
//	glEnd();
//
//};

