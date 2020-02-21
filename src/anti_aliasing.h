#pragma once

#include "refraction.h"

class AntiAliasing : public Refraction
{
public:
	AntiAliasing(short width, short height);
	virtual ~AntiAliasing();
	virtual void DrawScene();
};