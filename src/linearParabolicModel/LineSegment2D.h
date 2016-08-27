#pragma once

#include <float.h>
#include <iostream>
using std::cout;
#include "_aux.h"

class LineSegment2D
{
public:
	LineSegment2D(float XA=0.f, float YA=0.f,
				  float XB=0.f, float YB=0.f)
	{
		A.X = XA;
		A.Y = YA;

		B.X = XB;
		B.Y = YB;

		calcTerms();
	}
	void setVertexA( float X, float Y ){
		A.X = X;
		A.Y = Y;
		calcTerms();
	}
	void setVertexB( float X, float Y ){
		B.X = X;
		B.Y = Y;
		calcTerms();
	}
	inline Vertex2D getCoordAt( float u, float v ){
		Vertex2D pt;
		pt.X = A.X + u*(B.X - A.X);
		pt.Y = A.Y + v*(B.Y - A.Y);

		return pt;
	}
	inline float getM(void){	return m;	}
	inline float getIndependentTerm(void){	return indTerm;	}
	inline float getXAt(float y){	return (y-indTerm)/m;	}
	inline float getYAt(float x){	return (m*x)+indTerm;	}
	/* getCrossSect: retorna o ponto de intersecção entre esta reta
	 * e outra reta passada por parâmetro. Se a intersecção não existir,
	 * é retornado o vetor com Valor zerado. O ponto de intersecção é 
	 * retornado independente de ele estar dentro do escopo das retas.
	 * Para saber se a intersecção encontra-se no escopo das retas,
	 * deve ser chamada a função ''bool crossSectExists( )''.
	 *
	 */
	inline Vertex2D getCrossSect( LineSegment2D &line2 ){
		Vertex2D C = line2.A;
		Vertex2D D = line2.B;
		
		/*		A intersecção entre as retas é encontrada a partir da resolução 
		 *	de um sistema envolvendo a equação das duas retas em questão.
		 */

		float	a = (B.Y-A.Y)/(B.X-A.X),
//				b = -1,
				d = (D.Y-C.Y)/(D.X-C.X),
				e = -1;

		float	c = a*(-A.X) + A.Y,
				f = d*(-C.X) + C.Y;

		float x = 0.0, y = 0.0;

		if( (a-d) != 0 )
		{
			//a -= d;
			//c -= f;
			//x = -c/a;
			x = (-(c-f))/(a-d);
			y = (-f-(d*x))/e;
			//y = (-f-(d*x))/e;
		}

		Vertex2D ret;
		ret.X = x;
		ret.Y = y;

		return ret;
	}

private:
	inline void calcTerms(void){
		float deltaX = (B.X - A.X);
		if(deltaX == 0.0f) deltaX = FLT_MIN;
		m = (B.Y - A.Y)/deltaX;
		indTerm = B.Y-(m*B.X);
	}

	Vertex2D A;
	Vertex2D B;
	float m;
	float indTerm;
};
