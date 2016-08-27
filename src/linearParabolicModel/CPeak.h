#pragma once

class Peak
{
	int Angle;
	float Magnitude;
	public:
		int getAngle(){return Angle;}
		float getMagnitude(){return Magnitude;}
		void setAngle(int Ang){Angle = Ang;}
		void setMagnitude(float Mag){Magnitude = Mag;}		
};